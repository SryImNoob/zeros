# ZerOS Makefile
# Build and run a bare-metal AArch64 kernel for Raspberry Pi Zero 2 W

KERNEL_ELF = target/aarch64-unknown-none/release/zeros
KERNEL_IMG = kernel8.img
SDCARD_IMG = sdcard.img

# Load local git-ignored configuration (e.g. RPI_HOST)
-include local.mk

.PHONY: build sdimage clean qemu qemu-display objdump sdcard deploy tryboot log

# Build the kernel binary
build:
	cargo build --release
	$(shell find $$(rustc --print sysroot) -name "llvm-objcopy" | head -1) -O binary $(KERNEL_ELF) $(KERNEL_IMG)
	@echo ""
	@echo "✅ Built $(KERNEL_IMG)"
	@ls -lh $(KERNEL_IMG)

# Build virtual dual-partition FAT32 SD card image (Slot A + Slot B)
sdimage: build
	python3 scripts/mk_sdcard.py $(SDCARD_IMG)

# Run in QEMU (headless, UART only, with FAT32 SD card)
qemu: sdimage
	qemu-system-aarch64 \
		-M raspi3b \
		-kernel $(KERNEL_IMG) \
		-drive file=$(SDCARD_IMG),if=sd,format=raw \
		-serial stdio \
		-display none

# Run in QEMU with display window (framebuffer visible, with FAT32 SD card)
qemu-display: sdimage
	qemu-system-aarch64 \
		-M raspi3b \
		-kernel $(KERNEL_IMG) \
		-drive file=$(SDCARD_IMG),if=sd,format=raw \
		-serial stdio

# Disassemble the kernel for debugging
objdump: build
	$(shell find $$(rustc --print sysroot) -name "llvm-objdump" | head -1) -d $(KERNEL_ELF) | head -100

# Clean build artifacts
clean:
	cargo clean
	rm -f $(KERNEL_IMG) $(SDCARD_IMG)

# Copy kernel to mounted SD card (adjust MOUNT_POINT as needed)
MOUNT_POINT ?= /Volumes/BOOT
sdcard: build
	@if [ -d "$(MOUNT_POINT)" ]; then \
		cp $(KERNEL_IMG) $(MOUNT_POINT)/; \
		echo "✅ Copied $(KERNEL_IMG) to $(MOUNT_POINT)"; \
		diskutil eject $(MOUNT_POINT); \
	else \
		echo "❌ SD card not mounted at $(MOUNT_POINT)"; \
	fi

# Deploy zeros.img + tryboot.txt to Raspberry Pi over SSH (uses RPI_HOST from local.mk)
deploy: build
	@if [ -z "$(RPI_HOST)" ]; then \
		echo "❌ RPI_HOST is not set in local.mk"; exit 1; \
	fi
	scp $(KERNEL_IMG) $(RPI_HOST):/tmp/zeros.img
	ssh -t $(RPI_HOST) 'BOOT_DIR=/boot/firmware; [ ! -d "$$BOOT_DIR" ] && BOOT_DIR=/boot; \
		sudo cp /tmp/zeros.img $$BOOT_DIR/zeros.img && \
		if [ ! -f "$$BOOT_DIR/ZEROS.LOG" ]; then sudo dd if=/dev/zero bs=512 count=128 2>/dev/null | tr "\0" " " | sudo tee $$BOOT_DIR/ZEROS.LOG > /dev/null; fi && \
		if [ ! -f "$$BOOT_DIR/FW43430.BIN" ]; then \
			FW_SRC=$$(ls /lib/firmware/brcm/brcmfmac43430*-sdio.bin 2>/dev/null | head -1); \
			[ -n "$$FW_SRC" ] && sudo cp -L "$$FW_SRC" $$BOOT_DIR/FW43430.BIN; \
		fi && \
		if [ ! -f "$$BOOT_DIR/FW43430.TXT" ]; then \
			NV_SRC=$$(ls /lib/firmware/brcm/brcmfmac43430*-sdio*.txt 2>/dev/null | head -1); \
			[ -n "$$NV_SRC" ] && sudo cp -L "$$NV_SRC" $$BOOT_DIR/FW43430.TXT; \
		fi && \
		if [ ! -f "$$BOOT_DIR/WIFI.TXT" ]; then \
			SSID=$$(iwgetid -r 2>/dev/null || nmcli -t -f active,ssid dev wifi 2>/dev/null | grep "^yes:" | cut -d: -f2 | head -1); \
			PASS=$$(sudo grep -rhE "^(psk|wifi-sec\.psk)=" /etc/NetworkManager/system-connections/ /etc/wpa_supplicant/ 2>/dev/null | head -1 | cut -d= -f2- | tr -d "\""); \
			IP=$$(hostname -I 2>/dev/null | awk "{print \$$1}"); \
			GW=$$(ip route 2>/dev/null | awk "/^default/ {print \$$3; exit}"); \
			printf "SSID=%s\nPASS=%s\nIP=%s\nGW=%s\n" "$$SSID" "$$PASS" "$$IP" "$$GW" | sudo tee $$BOOT_DIR/WIFI.TXT > /dev/null; \
		fi && \
		python3 -c "import urllib.request, json; uks=[]; [uks.append(json.loads(urllib.request.urlopen(\"https://music.163.com/api/login/qrcode/unikey?type=1\", timeout=3).read()).get(\"unikey\",\"\")) for _ in range(6)]; print(\"\\n\".join([u for u in uks if len(u)==36]))" 2>/dev/null | sudo tee $$BOOT_DIR/UNIKEY.TXT > /dev/null || true && \
		printf "arm_64bit=1\nkernel=zeros.img\nkernel_address=0x80000\nenable_uart=1\ndtparam=spi=on\n" | sudo tee $$BOOT_DIR/tryboot.txt > /dev/null && \
		sync && \
		echo "✅ Deployed zeros.img, FW43430.BIN/TXT, WIFI.TXT, UNIKEY.TXT, and tryboot.txt to $$BOOT_DIR on $(RPI_HOST)"'

# Read persistent kernel blackbox log from Raspberry Pi SD card
log:
	ssh $(RPI_HOST) 'BOOT_DIR=/boot/firmware; [ ! -d "$$BOOT_DIR" ] && BOOT_DIR=/boot; sed "s/[[:space:]]*$$//" $$BOOT_DIR/ZEROS.LOG | grep -v "^$$"'

# Deploy and immediately one-shot reboot into ZerOS via tryboot
tryboot: deploy
	ssh -t $(RPI_HOST) 'sudo reboot "0 tryboot"' || true
	@echo "🚀 Rebooting $(RPI_HOST) into ZerOS (one-time tryboot)..."

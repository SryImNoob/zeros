# ZerOS Makefile
# Build and run a bare-metal AArch64 kernel for Raspberry Pi Zero 2 W

KERNEL_ELF = target/aarch64-unknown-none/release/zeros
KERNEL_IMG = kernel8.img
SDCARD_IMG = sdcard.img

# Load local git-ignored configuration (e.g. RPI_HOST)
-include local.mk

.PHONY: build sdimage clean qemu qemu-display objdump sdcard deploy tryboot

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
		printf "arm_64bit=1\nkernel=zeros.img\nkernel_address=0x80000\nenable_uart=1\ndtparam=spi=on\n" | sudo tee $$BOOT_DIR/tryboot.txt > /dev/null && \
		echo "✅ Deployed zeros.img and tryboot.txt to $$BOOT_DIR on $(RPI_HOST)"'

# Deploy and immediately one-shot reboot into ZerOS via tryboot
tryboot: deploy
	ssh -t $(RPI_HOST) 'sudo reboot "0 tryboot"' || true
	@echo "🚀 Rebooting $(RPI_HOST) into ZerOS (one-time tryboot)..."

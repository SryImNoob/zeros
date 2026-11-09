# ZerOS Makefile
# Build and run a bare-metal AArch64 kernel for Raspberry Pi Zero 2 W

KERNEL_ELF = target/aarch64-unknown-none/release/zeros
KERNEL_IMG = kernel8.img

.PHONY: build clean qemu objdump

# Build the kernel binary
build:
	cargo build --release
	$(shell find $$(rustc --print sysroot) -name "llvm-objcopy" | head -1) -O binary $(KERNEL_ELF) $(KERNEL_IMG)
	@echo ""
	@echo "✅ Built $(KERNEL_IMG)"
	@ls -lh $(KERNEL_IMG)

# Run in QEMU (headless, UART only)
qemu: build
	qemu-system-aarch64 \
		-M raspi3b \
		-kernel $(KERNEL_IMG) \
		-serial stdio \
		-display none

# Run in QEMU with display window (framebuffer visible)
qemu-display: build
	qemu-system-aarch64 \
		-M raspi3b \
		-kernel $(KERNEL_IMG) \
		-serial stdio

# Disassemble the kernel for debugging
objdump: build
	$(shell find $$(rustc --print sysroot) -name "llvm-objdump" | head -1) -d $(KERNEL_ELF) | head -100

# Clean build artifacts
clean:
	cargo clean
	rm -f $(KERNEL_IMG)

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

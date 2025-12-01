# Makefile for BMP280 Mesh Sensor Node

.PHONY: all build upload clean monitor

all: build upload

build:
	@echo "Building firmware..."
	pio run -e d1_mini

upload:
	@echo "Uploading firmware..."
	pio run -e d1_mini --target upload

monitor:
	@echo "Opening serial monitor..."
	pio device monitor -b 115200

clean:
	@echo "Cleaning build files..."
	pio run --target clean

rebuild: clean build

full: rebuild upload monitor

help:
	@echo "Available targets:"
	@echo "  make build      - Build firmware"
	@echo "  make upload     - Upload firmware to device"
	@echo "  make monitor    - Open serial monitor"
	@echo "  make clean      - Clean build files"
	@echo "  make rebuild    - Clean and rebuild"
	@echo "  make full       - Clean, build, upload, and monitor"

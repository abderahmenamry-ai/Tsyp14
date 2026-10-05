# Host builds of the firmware: unit tests + the shared library used by the software-in-the-loop tests.
CXX      ?= g++
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra
INC      = -Iinclude -Ihost -I../comms/firmware
BUILD    = build

all: $(BUILD)/libfw_sim.so $(BUILD)/test_core

$(BUILD)/libfw_sim.so: src/apps.cpp host/fw_sim.cpp host/sim_hal.h include/*.h ../comms/firmware/*.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -fPIC -shared $(INC) src/apps.cpp host/fw_sim.cpp -o $@

$(BUILD)/test_core: test/test_core.cpp include/*.h ../comms/firmware/*.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(INC) test/test_core.cpp -o $@

# syntax-check the ESP32/Arduino glue against minimal stubs (the real build is `pio run`)
check-esp32:
	$(CXX) -std=c++11 -fsyntax-only -Wall -Itools/stubs $(INC) -DFW_ROLE_WRITER src/hal_esp32.cpp src/main.cpp
	$(CXX) -std=c++11 -fsyntax-only -Wall -Itools/stubs $(INC) -DFW_ROLE_EXECUTOR src/hal_esp32.cpp src/main.cpp
	$(CXX) -std=c++11 -fsyntax-only -Wall -Itools/stubs $(INC) -DFW_ROLE_HUB src/hal_esp32.cpp src/main.cpp

test: all
	./$(BUILD)/test_core
	cd .. && python3 firmware/sil/test_fw.py

sil: all
	cd .. && python3 firmware/sil/run_sil.py --check
	cd .. && python3 firmware/sil/run_sil.py --loss 0.5 --check

clean:
	rm -rf $(BUILD)
.PHONY: all test sil clean check-esp32

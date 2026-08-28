BUILD_DIR := build/debug

HW06_CPP := homework_06/include/ballistics.hpp \
            homework_06/src/ballistics.cpp \
            homework_06/src/main.cpp \
            homework_06/tests/ballistics_tests.cpp

HW06_CMAKE := homework_06/CMakeLists.txt

# homework_07 vendors a single-header library (json.hpp) that is intentionally
# excluded from format/lint — only first-party sources are listed here.
HW07_CPP := homework_07/main.cpp

HW07_CMAKE := homework_07/CMakeLists.txt

# homework_08/09 навмисно не додані (legacy, написані до clang-tidy дисципліни).
# homework_10 написаний clang-tidy-clean; third_party/json.hpp не входить у скоуп.
HW10_CPP := $(wildcard homework_10/include/*.h) \
            $(wildcard homework_10/include/*/*.h) \
            $(wildcard homework_10/src/*.cpp) \
            $(wildcard homework_10/src/*/*.cpp)

HW10_CMAKE := homework_10/CMakeLists.txt

# homework_11 (UART+GPIO автопілот). LibgpiodGpio.cpp виключено з lint/format:
# при USE_GPIOD=OFF його нема в compile_commands.json, clang-tidy -p впав би.
HW11_CPP := $(wildcard homework_11/include/*.h) \
            $(wildcard homework_11/include/*/*.h) \
            $(filter-out homework_11/src/gpio/LibgpiodGpio.cpp, \
              $(wildcard homework_11/src/*.cpp) $(wildcard homework_11/src/*/*.cpp))

HW11_CMAKE := homework_11/CMakeLists.txt

# Aggregate of all first-party files; append new homeworks here.
CPP_FILES   := $(HW06_CPP) $(HW07_CPP) $(HW10_CPP) $(HW11_CPP)
CMAKE_FILES := $(HW06_CMAKE) $(HW07_CMAKE) $(HW10_CMAKE) $(HW11_CMAKE)

# clang-tidy запускаємо лише на translation units (.cpp); діагностику в хедерах
# дає HeaderFilterRegex під час обробки відповідного .cpp — як і в CI.
LINT_FILES := $(filter %.cpp,$(CPP_FILES))

HW10_BIN := $(BUILD_DIR)/homework_10/drone_sim_v4
HW11_BIN := $(BUILD_DIR)/homework_11/drone_autopilot

.PHONY: format lint test quality run-hw10 run-hw10-tsan run-hw11

# Apply clang-format and cmake-format to all first-party sources.
format:
	clang-format -i $(CPP_FILES)
	cmake-format -i $(CMAKE_FILES)

# Run clang-tidy locally, mirroring CI: configure (генерує compile_commands.json),
# build (ловить compile-помилки), потім clang-tidy на .cpp. Запускати в devcontainer.
lint:
	cmake --preset debug
	cmake --build --preset debug
	clang-tidy -p $(BUILD_DIR) $(LINT_FILES)

# Run all registered CTest tests.
test:
	ctest --test-dir $(BUILD_DIR) --output-on-failure

# Format, then lint, then test — run this before every PR.
quality: format lint test

# Зібрати і запустити homework_10 (3 потоки). simulation.json пишеться у homework_10/.
run-hw10:
	cmake --preset debug -DDRONE_SIM_TSAN=OFF
	cmake --build --preset debug --target drone_sim_v4
	cd homework_10 && ../$(HW10_BIN)

# Те саме під ThreadSanitizer — перевірка гонок даних (Linux/WSL).
run-hw10-tsan:
	cmake --preset debug -DDRONE_SIM_TSAN=ON
	cmake --build --preset debug --target drone_sim_v4
	cd homework_10 && TSAN_OPTIONS=halt_on_error=0 ../$(HW10_BIN)

# Зібрати homework_11 (UART+GPIO автопілот) і запустити з дефолтними аргументами.
# Потрібен запущений чекер + socat/gpio-sim (див. homework_11/scripts/run_sim.sh).
run-hw11:
	cmake --preset debug
	cmake --build --preset debug --target drone_autopilot
	cd homework_11 && ../$(HW11_BIN)

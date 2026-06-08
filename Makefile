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

# Aggregate of all first-party files; append new homeworks here.
CPP_FILES   := $(HW06_CPP) $(HW07_CPP)
CMAKE_FILES := $(HW06_CMAKE) $(HW07_CMAKE)

.PHONY: format lint test quality

# Apply clang-format and cmake-format to all first-party sources.
format:
	clang-format -i $(CPP_FILES)
	cmake-format -i $(CMAKE_FILES)

# Run clang-tidy against the debug compile_commands.json.
# Requires a successful cmake --preset debug run first.
lint:
	clang-tidy -p $(BUILD_DIR) $(CPP_FILES)

# Run all registered CTest tests.
test:
	ctest --test-dir $(BUILD_DIR) --output-on-failure

# Format, then lint, then test — run this before every PR.
quality: format lint test

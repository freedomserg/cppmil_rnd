BUILD_DIR := build/debug

HW06_CPP := homework_06/include/ballistics.hpp \
            homework_06/src/ballistics.cpp \
            homework_06/src/main.cpp \
            homework_06/tests/ballistics_tests.cpp

HW06_CMAKE := homework_06/CMakeLists.txt

.PHONY: format lint test quality

# Apply clang-format and cmake-format to homework_06 files.
format:
	clang-format -i $(HW06_CPP)
	cmake-format -i $(HW06_CMAKE)

# Run clang-tidy against the debug compile_commands.json.
# Requires a successful cmake --preset debug run first.
lint:
	clang-tidy -p $(BUILD_DIR) $(HW06_CPP)

# Run all registered CTest tests.
test:
	ctest --test-dir $(BUILD_DIR) --output-on-failure

# Format, then lint, then test — run this before every PR.
quality: format lint test

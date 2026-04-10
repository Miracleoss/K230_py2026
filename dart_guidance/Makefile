# 飞镖制导库 Makefile

# 编译器设置
CC = gcc
CFLAGS = -Wall -Wextra -O2 -I./include
DEBUG_CFLAGS = -Wall -Wextra -g -O0 -I./include

# 目标文件
LIB_SRCS = src/guidance_math.c src/guidance_core.c src/dart_guidance.c
LIB_OBJS = $(LIB_SRCS:.c=.o)
LIB_NAME = libdart_guidance.a

# 示例程序
EXAMPLE_SRC = examples/simple_example.c
EXAMPLE_EXE = examples/simple_example

# 测试程序
TEST_SRC = test/test_basic.c
TEST_EXE = test/test_basic

# 默认目标
all: lib example test

# 编译静态库
lib: $(LIB_OBJS)
	ar rcs $(LIB_NAME) $(LIB_OBJS)
	@echo "静态库 $(LIB_NAME) 创建成功"

# 编译目标文件
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# 编译示例程序
example: lib
	$(CC) $(CFLAGS) $(EXAMPLE_SRC) -L. -ldart_guidance -lm -o $(EXAMPLE_EXE)
	@echo "示例程序 $(EXAMPLE_EXE) 编译成功"

# 编译测试程序
test: lib
	$(CC) $(CFLAGS) $(TEST_SRC) -L. -ldart_guidance -lm -o $(TEST_EXE)
	@echo "测试程序 $(TEST_EXE) 编译成功"

# 调试版本
debug: CFLAGS = $(DEBUG_CFLAGS)
debug: clean all

# 运行示例
run-example: example
	@echo "运行示例程序..."
	./$(EXAMPLE_EXE)

# 运行测试
run-test: test
	@echo "运行测试程序..."
	./$(TEST_EXE)

# 清理
clean:
	rm -f $(LIB_OBJS) $(LIB_NAME) $(EXAMPLE_EXE) $(TEST_EXE)
	@echo "清理完成"

# 安装库（系统目录）
install: lib
	cp $(LIB_NAME) /usr/local/lib/
	cp include/*.h /usr/local/include/
	@echo "库安装完成"

# 卸载
uninstall:
	rm -f /usr/local/lib/$(LIB_NAME)
	rm -f /usr/local/include/dart_guidance.h
	rm -f /usr/local/include/guidance_types.h
	rm -f /usr/local/include/guidance_math.h
	rm -f /usr/local/include/guidance_core.h
	@echo "库卸载完成"

# 帮助信息
help:
	@echo "可用命令:"
	@echo "  make all        - 编译所有目标（库、示例、测试）"
	@echo "  make lib        - 只编译静态库"
	@echo "  make example    - 编译示例程序"
	@echo "  make test       - 编译测试程序"
	@echo "  make run-example - 编译并运行示例程序"
	@echo "  make run-test   - 编译并运行测试程序"
	@echo "  make debug      - 编译调试版本"
	@echo "  make clean      - 清理编译文件"
	@echo "  make install    - 安装库到系统目录"
	@echo "  make uninstall  - 卸载库"
	@echo "  make help       - 显示此帮助信息"

.PHONY: all lib example test debug run-example run-test clean install uninstall help

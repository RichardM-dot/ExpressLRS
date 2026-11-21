/*
 * ============================================================================
 * 【中文导读】test_main.cpp —— native 单元测试环境的空 main()
 * ----------------------------------------------------------------------------
 * 文件职责：
 *   当 platformio.ini 的 [env:native] 以“可执行程序”方式构建库/单测时，
 *   链接器需要一个 main()。本文件提供最小实现：打印一行提示后返回 0。
 *   真正在跑的 main() 是 test/ 目录下由 Unity 测试框架生成的版本，
 *   因此本文件用 #ifndef UNIT_TEST 包起来，避免符号冲突。
 *
 * 主要函数：
 *   main() —— 空实现，仅满足链接需求
 * ============================================================================
 */
#ifndef UNIT_TEST
#include <iostream>

using namespace std;

int main()
{
	cout << "Doing nothing just keep build all happy!" << endl;
	return 0;
}
#endif

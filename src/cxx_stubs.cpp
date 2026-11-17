/*
 * frank-kickstart - the C++ runtime's verbose terminate handler (which
 * pulls in the 30 KB name demangler) replaced by a plain abort: the PS/2
 * driver is C++, and the launcher has 256 KB of flash.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <cstdlib>

namespace __gnu_cxx {
void __verbose_terminate_handler() { std::abort(); }
}

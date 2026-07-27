// Shim for <conio.h> so the Windows-oriented simulator builds on Linux.
// The simulator uses no conio functions, but MSVC's standard headers pull in
// <memory>/<limits>/<ctime> transitively while libstdc++ does not — provide
// them here since simulator.h includes <conio.h> before using shared_ptr etc.
#pragma once
#include <memory>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <string>

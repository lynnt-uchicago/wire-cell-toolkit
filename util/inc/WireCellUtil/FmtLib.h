/**
   Recent SPDLOG can be compiled with newer, external fmtlib and that no longer
   uses ostream insertion operator without us making a fmt::formatter<>.  OTOH,
   this form doesn't exist in older fmtlib such as included in SPDLOG.

   So, here define a dummy formatter base when old or spdlog fmtlib is used.

   See https://github.com/WireCell/wire-cell-spack/issues/12

 */

#ifndef WIRECELL_FMTLIB_H
#define WIRECELL_FMTLIB_H

#ifdef SPDLOG_FMT_EXTERNAL
#include <fmt/core.h>
#if FMT_VERSION >= 90000
#include <fmt/ostream.h>
#else
namespace fmt {
    struct ostream_formatter {};
    template<typename T> struct formatter {};
}
#endif
#else
namespace fmt {
    struct ostream_formatter {};
    template<typename T> struct formatter {};
}
#endif

#endif

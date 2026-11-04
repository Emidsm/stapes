# Build flags shared by every Stapes target, exposed as stapes::options.

add_library(stapes_options INTERFACE)
add_library(stapes::options ALIAS stapes_options)

target_compile_options(
  stapes_options
  INTERFACE -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wsign-conversion
            -Wshadow
            -Wold-style-cast
            -Wcast-align
            -Woverloaded-virtual
            -Wnon-virtual-dtor
            -Wdouble-promotion
            -Wformat=2
            -Wimplicit-fallthrough
            # Determinism: a fused multiply-add rounds differently from a separate
            # multiply and add, so the compiler must not fuse them on its own.
            -ffp-contract=off)

if(STAPES_WERROR)
  target_compile_options(stapes_options INTERFACE -Werror)
endif()

if(STAPES_COVERAGE)
  target_compile_options(stapes_options INTERFACE --coverage)
  target_link_options(stapes_options INTERFACE --coverage)
endif()

# Sanitizers apply to the whole build, dependencies included, so that a report
# crossing into GoogleTest or Eigen code is complete.
if(STAPES_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=all
                      -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

Compact C++ Z80 emulator
========================
An experiment on combining C++11 templates and preprocessor to
write a full Z80 interpreter with optimal speed *and* very very
compact but maintainable code.

This is still work-in-progress, it's mostly a playfield for me
to experiment different techniques. Currently, I'm partly satisfied
with the code compactness, but it's still not where I would like it
to be.

Compilation times are crazy (2 minutes with LLVM 5.1 on i5 2Ghz,
even at -O0), it's probably some quadratic behavior in the clang
frontend.

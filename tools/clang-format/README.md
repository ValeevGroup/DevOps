# clang-format utils
- `clang-format.sh`: wrapper that restricts version and uses docker if supported version is not found
- `clang-format-input.cpp` and `clang-format-output-{13..17}.cpp`: reference input and outputs (using `--style=Google`)
  - note that using `--style=LLVM` produces identical outputs for all versions, but Google style is preferred from spot polling

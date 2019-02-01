# Piksi Firmware development

PFWP uses clang-format for auto-formatting all source files. The configuration is stored as `.clang-format` at the repository's root. We currently use clang-format 6.0

## Installing clang-format

### Debian based distributions
Follow the instructions on http://apt.llvm.org to configure your package manager to fetch llvm packages. Then, install `clang-format-6.0`

### macOS
`brew install clang-format`

## Using clang-format
We provide two Makefile targets:

* `clang-format-all`: Formats all c source files under `src/`. You should avoid using this target since it will auto-format everything and might cause merge conflicts.
* `clang-format-head`: Formats the changed lines of all staged files. You can review the changes using `git diff` and use `git add` to stage the formatting changes. This is the preferred workflow since it will avoid silly commits which do nothing except changing the formatting.
* `clang-format-diff` Formats all lines of source which differ from master. Use this before submitting a pull-request if you were lazy and forgot to `clang-format-head` appropriately.

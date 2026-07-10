# Image_Process

Image processing / artwork-generation module. Each subfolder is one
generation algorithm ("path"), self-contained with its own scripts, tests,
and README:

```text
Image_Process/
  mondrian/    Recursive-subdivision Mondrian-style line/block artwork
               generator + path generator (see mondrian/README.md).
```

More algorithms (e.g. converting arbitrary source images into painting
plans) are planned as additional sibling folders here — each new approach
gets its own subfolder rather than growing inside `mondrian/`.

`mondrian/` still reads its config profiles from the repo-root `configs/`
directory and writes to the repo-root `output/` directory (both are
CWD-relative paths in the scripts, so they're run from the repo root — see
`mondrian/README.md`).

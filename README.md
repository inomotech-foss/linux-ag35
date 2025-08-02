# Linux AG35

Quectel source: AG35EVDR08A13T4G_OCPU_01.001.02

Using upstream source: <https://git.codelinaro.org/clo/la/kernel/msm-3.18/-/tags/LE.UM.2.3.6.c7-00100-9x07>

Manually rebased changes from Quectel source onto `4b5ee0855cacb2456cc2de9950dd8a9b700449da`.

This commit was determined with

```bash
for hash in $(git log --pretty=format:%H LE.UM.2.3.6.c7-00100-9x07); do echo "$hash: $(git diff HEAD..$hash --stat | wc -l)"; done
```

Split changes into multiple commits.


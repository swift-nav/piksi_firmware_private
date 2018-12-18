# How to profile ME

In the root directory of PFW:

```
mkdir build
cd build
meson .. --buildtype debugoptimized
ninja
valgrind --tool=callgrind ./mesta
kcachegrind callgrind.out.XXXX
```

For more details:
https://baptiste-wicht.com/posts/2011/09/profile-c-application-with-callgrind-kcachegrind.html

# How to run static analysis tool

In the root directory of PFW:

```
mkdir build
cd build
meson ..
ninja scan-build
```

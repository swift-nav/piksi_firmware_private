# How to profile ME

In the root directory of PFW:

```
cd mesta
make mesta
valgrind --tool=callgrind ./mesta/mesta
kcachegrind callgrind.out.XXXX
```

For more details:
https://baptiste-wicht.com/posts/2011/09/profile-c-application-with-callgrind-kcachegrind.html

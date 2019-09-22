
package(default_visibility = ["//visibility:public"])

cc_binary(
  name = "bark.so",
  srcs = glob(["*.cc", "*.h"),
  deps = [
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
    "//src/commons:parameters",
    "//src/dynamics:dynamics",
    "//src:optimizer"
  ],
  linkshared = 1,
  linkstatic = 1
)
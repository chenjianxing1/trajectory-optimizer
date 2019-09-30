cc_library(
  name = "parameters",
  hdrs = ["parameters.h"],
	visibility = ["//visibility:public"]
)

cc_library(
  name = "commons",
  hdrs = ["commons.h"],
  deps = ["//src/geometry:geometry",
          ":parameters"],
	visibility = ["//visibility:public"]
)

py_library(
  name = "py_commons",
  srcs = ["py_commons.py"],
  data = ["//python:optimizer.so"],
  imports = ["../../python/"],
	visibility = ["//visibility:public"]
)
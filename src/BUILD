cc_library(
  name = "optimizer",
  hdrs = glob(["*.h"]),
  srcs = glob(["*.cc"]),
  deps = [
    "//src/commons:parameters",
    "//src/geometry:geometry",
    "@com_google_ceres_solver//:ceres"
  ],
	visibility = ["//visibility:public"]
)
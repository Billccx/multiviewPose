cc_library(
  name = "eigen",
  hdrs = glob(["include/eigen3/**","include/eigen3/Eigen/**"]),
  includes = ["include/eigen3","include/eigen3/Eigen",],
  srcs = [],
  visibility = ["//visibility:public"],
  linkstatic = 1,
)


# CERES_DEFINES = [
#     "CERES_USE_CXX14",
#     "CERES_USE_EIGEN_SPARSE",
#     "CERES_NO_SUITESPARSE",
#     "CERES_NO_LAPACK",
#     "CERES_NO_CXSPARSE",
#     "CERES_STD_UNORDERED_MAP",
#     "CERES_USE_CXX14_THREADS",

#     # Use the internal mutex code. Not ideal, but it works.
#     "CERES_HAVE_PTHREAD",
#     "CERES_HAVE_RWLOCK",
# ]

# cc_library(
#   name = "ceres",
#   hdrs = glob(["local/include/ceres/**"]),
#   srcs = ["local/lib/libceres.a",
#   "lib/x86_64-linux-gnu/libcxsparse.a",
#   "lib/x86_64-linux-gnu/libcxsparse.so",
#   "lib/x86_64-linux-gnu/libcxsparse.so.3",
#   "lib/x86_64-linux-gnu/libcxsparse.so.3.2.0",
#   "lib/x86_64-linux-gnu/libsuitesparseconfig.a",
#   "lib/x86_64-linux-gnu/libsuitesparseconfig.so",
#   "lib/x86_64-linux-gnu/libsuitesparseconfig.so.5",
#   "lib/x86_64-linux-gnu/libsuitesparseconfig.so.5.7.1",
#   ],
#   includes = ["include/ceres",],
#   defines = CERES_DEFINES,
#   visibility = ["//visibility:public"],
#   linkstatic = 1,
# )
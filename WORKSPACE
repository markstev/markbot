load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

# GoogleTest/GoogleMock framework. Used by most unit-tests.
http_archive(
     name = "google_googletest",
     urls = ["https://github.com/google/googletest/archive/b6cd405286ed8635ece71c72f118e659f4ade3fb.zip"],  # 2019-01-07
     strip_prefix = "googletest-b6cd405286ed8635ece71c72f118e659f4ade3fb",
     sha256 = "ff7a82736e158c077e76188232eac77913a15dac0b22508c390ab3f88e6d6d86",
)

git_repository(
		name = "markstev_tensixty",
		remote = "http://github.com/markstev/tensixty.git",
    commit = "bf640054ff5854c4dee2112b5b7294666f8886b0",
)

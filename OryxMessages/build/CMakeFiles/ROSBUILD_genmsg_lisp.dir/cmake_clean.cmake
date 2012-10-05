FILE(REMOVE_RECURSE
  "../src/OryxMessages/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Temperature.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Temperature.lisp"
  "../msg_gen/lisp/Battery.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Battery.lisp"
  "../msg_gen/lisp/BlobList.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_BlobList.lisp"
  "../msg_gen/lisp/Blob.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Blob.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

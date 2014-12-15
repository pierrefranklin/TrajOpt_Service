FILE(REMOVE_RECURSE
  "CMakeFiles/octomap_server_gencfg"
  "devel/include/octomap_server/OctomapServerConfig.h"
  "devel/share/octomap_server/docs/OctomapServerConfig.dox"
  "devel/share/octomap_server/docs/OctomapServerConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/octomap_server/cfg/OctomapServerConfig.py"
  "devel/share/octomap_server/docs/OctomapServerConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/octomap_server_gencfg.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

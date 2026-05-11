已加固定脚本：[tool/view_path_json.sh](/home/user/code/cpp-2026spring/final/tool/view_path_json.sh)。

用法：

```bash
./tool/view_path_json.sh
```

默认构建并打开：

```text
output/result.json
```

查看 Reeds-Shepp 测试输出：

```bash
./tool/view_path_json.sh output/reeds_shepp_empty_map_test.json --list
./tool/view_path_json.sh output/reeds_shepp_empty_map_test.json --path map_start_to_goal
```

我已经验证过脚本可以正常构建目标并转发参数。
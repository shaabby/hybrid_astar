#pragma once

#include <string>

/**
 * @brief HTML 单文件生成器。
 *
 * 将 JSON 数据嵌入 HTML 模板，生成可直接在浏览器中打开的单文件 demo。
 */
class HtmlWriter {
public:
    /**
     * @brief 将 JSON 数据包装为完整 HTML 页面。
     * @param[in] json 规划结果 JSON 字符串
     * @return 完整 HTML 文档字符串
     */
    [[nodiscard]] static std::string wrap(const std::string& json);
};

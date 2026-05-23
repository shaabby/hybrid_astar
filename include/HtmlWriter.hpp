/**
 * @file HtmlWriter.hpp
 * @brief HTML可视化包装器定义
 *
 * 提供将规划结果JSON数据嵌入HTML页面的功能，
 * 生成可在浏览器中查看的交互式可视化。
 */

#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <string>

/**
 * @brief HTML可视化数据包装器
 *
 * 将JSON格式的规划结果嵌入HTML模板，
 * 生成独立的HTML文件用于浏览器可视化展示。
 */
class HtmlWriter {
public:
    /**
     * @brief 将JSON数据包装为完整HTML文档
     * @param[in] json 规划结果的JSON字符串
     * @return 完整的HTML文档字符串
     */
    [[nodiscard]] static std::string wrap(const std::string& json);
};
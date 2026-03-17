# Changelog for package faceid

## 1.1.0 (2026-03-17)
------------------
1. 新增 RDK S100 适配。

## 1.0.0 (2026-03-13)
------------------
### 功能改进
1. 修复编译错误：移除 feature_manage.cpp 中的重复函数定义
2. 修复 const 正确性：在 database.h 和 database.cpp 中为 getItemCount() 添加 const 修饰符
3. 修复 CMakeLists.txt：添加 lsh_index.cpp 到源文件列表
4. 修复模型文件路径：faceid_node.cpp 现在正确使用 config/faceID.hbm 而不是 config/faceid.bin
5. 支持多模型适配：统一接口支持不同模型类型（int32、float）
6. 改进匹配策略：多特征融合、自适应阈值、间隙验证
7. 支持 X5 平台交叉编译：修复 cross-compilation 错误，确保可以在 RDK X5 上正常运行

### 问题修复
- 修复 faceid 功能包在 X5 平台交叉编译时报错的问题
- 修复模型文件路径配置错误导致无法加载模型的问题

tros_0.1.0 (2025-12-16)
------------------
1. 新增对人脸检测框输入、图片输入进行时间戳对齐功能。
2. 新增 faceid 模型人体特征提取能力。
3. 新增SQlite数据库对 faceid 提取的特征进行存储、管理、查询的能力。
4. 新增对新特征保持原始人物图片的能力, 用于可视化定位问题。

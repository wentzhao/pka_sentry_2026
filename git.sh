#!/bin/bash

# 提交代码到GitHub的自动化脚本

# 检查是否是Git仓库
if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "错误：当前目录不是一个Git仓库。"
    exit 1
fi

# 添加所有更改到暂存区
echo "正在添加文件更改..."
if ! git add . ; then
    echo "错误：添加文件到暂存区失败！"
    exit 1
fi

# 获取提交信息
commit_message="自动提交"
if [ -n "$1" ]; then
    commit_message="$1"
else
    read -p "请输入提交信息（留空使用默认信息）: " user_input
    [ -n "$user_input" ] && commit_message="$user_input"
fi

# 执行提交操作
echo "正在提交更改..."
if ! git commit -m "$commit_message" ; then
    echo "错误：提交失败！可能原因：没有需要提交的更改。"
    exit 1
fi

# 获取当前分支
current_branch=$(git symbolic-ref --short HEAD)
if [ -z "$current_branch" ]; then
    echo "错误：无法获取当前分支名称！"
    exit 1
fi

# 推送到远程仓库
echo "正在推送更改到 origin/$current_branch..."
if ! git push origin "$current_branch" ; then
    echo "错误：推送代码失败！"
    exit 1
fi

echo "✅ 成功将代码推送到GitHub！分支：$current_branch"

# git branch -m main

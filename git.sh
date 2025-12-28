# #!/bin/bash

# ============================================
# Git 自动提交脚本
# 用法: ./git_push.sh [提交信息]
# ============================================

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# 检查是否是Git仓库
check_git_repo() {
    if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        print_error "当前目录不是一个Git仓库。"
        exit 1
    fi
}

# 检查远程仓库
check_remote_repo() {
    if ! git remote | grep -q origin; then
        print_warning "没有配置远程仓库 origin。"
        read -p "请输入远程仓库URL（例如：https://github.com/username/repo.git）: " remote_url
        if [ -z "$remote_url" ]; then
            print_error "必须提供远程仓库URL。"
            exit 1
        fi
        git remote add origin "$remote_url"
        print_info "已添加远程仓库: $remote_url"
    fi
}

# 检查是否有未提交的更改
check_changes() {
    if [ -z "$(git status --porcelain)" ]; then
        print_warning "没有检测到任何更改。"
        exit 0
    fi
}

# 显示更改状态
show_changes() {
    print_info "当前更改状态:"
    echo "================================="
    git status --short
    echo "================================="
}

# 添加文件到暂存区
stage_changes() {
    print_info "正在添加文件到暂存区..."
    
    # 询问是否查看更改详情
    read -p "是否要查看文件更改详情？(y/N): " view_diff
    if [[ "$view_diff" =~ ^[Yy]$ ]]; then
        git diff --name-only
    fi
    
    # 提供选择：添加所有或选择添加
    read -p "添加所有更改？(Y/n): " add_all
    if [[ "$add_all" =~ ^[Nn]$ ]]; then
        print_info "请手动添加文件（输入文件路径，多个文件用空格分隔，或输入q退出）："
        while true; do
            read -p "文件: " files
            if [ "$files" = "q" ]; then
                print_info "退出文件添加。"
                exit 0
            fi
            if git add $files 2>/dev/null; then
                print_success "已添加文件: $files"
                break
            else
                print_error "添加文件失败，请重新输入。"
            fi
        done
    else
        if ! git add . ; then
            print_error "添加文件到暂存区失败！"
            exit 1
        fi
        print_success "已添加所有更改。"
    fi
}

# 获取提交信息
get_commit_message() {
    local default_msg="自动提交 - $(date '+%Y-%m-%d %H:%M:%S')"
    local commit_msg="$default_msg"
    
    if [ -n "$1" ]; then
        commit_msg="$1"
    else
        show_changes
        echo ""
        print_info "请输入提交信息："
        echo "（按Enter使用默认信息: $default_msg）"
        read -p "> " user_input
        
        if [ -n "$user_input" ]; then
            commit_msg="$user_input"
        fi
    fi
    
    echo "$commit_msg"
}

# 执行提交
commit_changes() {
    local msg="$1"
    print_info "正在提交更改..."
    
    if ! git commit -m "$msg" ; then
        print_error "提交失败！"
        exit 1
    fi
    
    print_success "提交成功！信息: $msg"
}

# 推送更改
push_changes() {
    local current_branch=$(git symbolic-ref --short HEAD 2>/dev/null || echo "main")
    
    if [ -z "$current_branch" ]; then
        current_branch="main"
    fi
    
    print_info "当前分支: $current_branch"
    print_info "正在推送更改到 origin/$current_branch..."
    
    # 检查是否需要设置上游分支
    if ! git branch -r | grep -q "origin/$current_branch"; then
        print_warning "远程分支 origin/$current_branch 不存在，首次推送需要设置上游分支。"
        if git push --set-upstream origin "$current_branch"; then
            print_success "首次推送成功并设置上游分支！"
        else
            print_error "推送失败！"
            exit 1
        fi
    else
        if git push origin "$current_branch"; then
            print_success "推送成功！"
        else
            print_error "推送失败！尝试强制推送？(y/N): "
            read force_push
            if [[ "$force_push" =~ ^[Yy]$ ]]; then
                if git push --force-with-lease origin "$current_branch"; then
                    print_success "强制推送成功！"
                else
                    print_error "强制推送失败！"
                    exit 1
                fi
            else
                exit 1
            fi
        fi
    fi
}

# 显示推送后的状态
show_final_status() {
    print_info "最终状态:"
    echo "================================"
    git log --oneline -3
    echo "================================"
    print_success "✅ 所有操作完成！"
}

# 主函数
main() {
    print_info "开始Git提交流程..."
    
    # 1. 检查Git仓库
    check_git_repo
    
    # 2. 检查远程仓库
    check_remote_repo
    
    # 3. 检查更改
    check_changes
    
    # 4. 显示更改
    show_changes
    
    # 5. 添加文件
    stage_changes
    
    # 6. 获取提交信息
    local commit_msg=$(get_commit_message "$1")
    
    # 7. 提交
    commit_changes "$commit_msg"
    
    # 8. 推送
    push_changes
    
    # 9. 显示最终状态
    show_final_status
}

# 执行主函数
main "$@"
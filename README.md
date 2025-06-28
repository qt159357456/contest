# github基本命令
* git branch：查看本地所有分支，-a查看远程分支
* git branch XXX：创建指定分支，git branch  -d XXX删除分支
* git checkout XXX：切换到指定分支
* git add . ：将当前所有文件添加进暂存区
* git commit -m "XXX，可中文"：将暂存区的文件进行提交
* git fetch origin：获取远程的最新更新，在上传远程前记得使用
* git push -u origin XXX：将当前所处分支上传到远程指定XXX分支，一般相同
* git merge XXX：将XXX分支合并到当前分支，发生冲突后需要手动修改
* git remote prune origin：清除本地不再使用的远程分支
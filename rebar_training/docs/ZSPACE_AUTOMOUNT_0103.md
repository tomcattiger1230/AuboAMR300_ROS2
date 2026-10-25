# 103的Zspace开机自动挂载

作用仅限arnold-NUC16JNK9XX11的现有CIFS共享和/mnt/zspace。脚本已部署，普通用户只读预检及实际systemd 259生成器校验通过；尚未执行root安装，当前/etc/fstab没有被改写。

在Mac终端执行一次（也可在103本机终端直接运行sudo后面的命令）：

```sh
ssh -t arnold@192.168.0.103 \
  'sudo python3 /home/arnold/Develop/rebar-training-resume-0103-20261022/rebar_training/scripts/configure_zspace_automount.py'
```

在终端内依次输入103的sudo密码及NAS密码，输入不回显，不需要发到聊天里。NAS用户名和共享地址取自当前已挂载配置。认证文件仅保存在103的/etc/samba/zspace.credentials，root所有、0600权限，不写进仓库或迁移包；fstab只引用文件路径。

安装先以独立只读CIFS连接验证认证，再备份原fstab并原子写入唯一目标条目；保留当前共享的协议版本、uid/gid及文件/目录模式。_netdev、nofail、x-systemd.automount、30秒挂载超时，使开机启用按需挂载，不要求NAS在开机瞬间可达。现有挂载不会被卸载或重启；下次开机后访问/mnt/zspace内的内容即可自动挂载，成功后保持连接。

只执行systemctl daemon-reload及生成单元校验；失败时恢复原fstab。原始备份和完整安装凭据保存在root专用/var/lib/zspace-automount，公开的不含密码凭据在/etc/zspace-automount.status.json。脚本不重启主机、不修改SSH/电源、不启动仿真或训练。

安装后可只读检查：

```sh
cat /etc/zspace-automount.status.json
systemctl show mnt-zspace.automount -p LoadState -p FragmentPath
findmnt --fstab --target /mnt/zspace
```

当前仍为人工挂载时automount可能尚未active，这是保持现有挂载不受影响的预期情况；生成的remote-fs.target.wants依赖负责下次开机启用。未执行重启验证前，不宣称已经验证重启实效。

如果已有认证文件需要更换密码，再运行同一脚本加--replace-credentials；旧文件只在103保留为root专用备份。不要把认证文件加入迁移归档，新主机应重新配置自己的认证。

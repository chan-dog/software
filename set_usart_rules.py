#!/usr/bin/env python3
import os
import subprocess
import re
import sys

def get_usb_serial_devices():
    """获取当前连接的USB串口设备列表"""
    devices = []
    
    try:
        # 查找所有ttyUSB和ttyACM设备
        tty_devices = []
        for dev in os.listdir('/dev'):
            if dev.startswith('ttyUSB') or dev.startswith('ttyACM'):
                tty_devices.append(dev)
        
        for dev in tty_devices:
            dev_path = f'/dev/{dev}'
            # 获取设备信息
            result = subprocess.run(
                ['udevadm', 'info', '--name', dev_path],
                capture_output=True,
                text=True
            )
            
            if result.returncode != 0:
                continue
            
            output = result.stdout
            # 提取关键信息
            id_serial = re.search(r'ID_SERIAL=(.*)', output)
            id_vendor = re.search(r'ID_VENDOR=(.*)', output)
            id_model = re.search(r'ID_MODEL=(.*)', output)
            dev_links = re.search(r'DEVLINKS=(.*)', output)
            
            if id_serial:
                device_info = {
                    'dev_path': dev_path,
                    'id_serial': id_serial.group(1),
                    'id_vendor': id_vendor.group(1) if id_vendor else 'N/A',
                    'id_model': id_model.group(1) if id_model else 'N/A',
                    'dev_links': dev_links.group(1) if dev_links else 'N/A'
                }
                devices.append(device_info)
        
        return devices
    
    except Exception as e:
        print(f"获取设备信息时出错: {e}")
        return []

def generate_udev_rule(device, bind_name, permissions='0666'):
    """生成udev规则并写入文件"""
    rule_file = '/etc/udev/rules.d/99-usb-serial-custom.rules'
    
    # 规则内容
    rule = f'SUBSYSTEM=="tty", ENV{{ID_SERIAL}}=="{device["id_serial"]}", SYMLINK+="{bind_name}", MODE="{permissions}"\n'
    
    try:
        # 检查规则是否已存在
        existing_rules = []
        if os.path.exists(rule_file):
            with open(rule_file, 'r') as f:
                existing_rules = f.readlines()
        
        # 如果规则已存在则替换，否则添加
        rule_exists = False
        with open(rule_file, 'w') as f:
            for r in existing_rules:
                if f'ENV{{ID_SERIAL}}=="{device["id_serial"]}"' in r:
                    # 替换现有规则
                    f.write(rule)
                    rule_exists = True
                else:
                    f.write(r)
            
            if not rule_exists:
                f.write(rule)
        
        print(f"规则已 {'更新' if rule_exists else '添加'} 到 {rule_file}")
        return True
    
    except PermissionError:
        print(f"权限不足，无法写入规则文件。请使用sudo运行脚本。")
        return False
    except Exception as e:
        print(f"写入规则文件时出错: {e}")
        return False

def reload_udev_rules():
    """重新加载udev规则"""
    try:
        subprocess.run(['sudo', 'udevadm', 'control', '--reload-rules'], check=True)
        subprocess.run(['sudo', 'udevadm', 'trigger'], check=True)
        print("udev规则已重新加载并生效")
        return True
    except subprocess.CalledProcessError as e:
        print(f"重新加载udev规则时出错: {e}")
        return False

def main():
    print("USB串口设备udev规则自动生成工具")
    print("=" * 40)
    
    # 获取设备列表
    devices = get_usb_serial_devices()
    
    if not devices:
        print("未找到任何USB串口设备")
        sys.exit(1)
    
    # 显示设备列表
    print("找到以下USB串口设备:")
    for i, device in enumerate(devices, 1):
        print(f"\n设备 {i}:")
        print(f"  设备路径: {device['dev_path']}")
        print(f"  厂商: {device['id_vendor']}")
        print(f"  型号: {device['id_model']}")
        print(f"  序列号: {device['id_serial']}")
    
    # 选择设备
    try:
        choice = int(input("\n请输入要绑定的设备编号 (1-{}): ".format(len(devices)))) - 1
        if choice < 0 or choice >= len(devices):
            print("无效的选择")
            sys.exit(1)
        selected_device = devices[choice]
    except ValueError:
        print("请输入有效的数字")
        sys.exit(1)
    
    # 获取用户输入的绑定参数
    bind_name = input("请输入绑定名称 (例如: usb2ttl_sensor): ")
    if not bind_name:
        print("绑定名称不能为空")
        sys.exit(1)
    
    permissions = input("请输入权限 (默认: 0666): ") or '0666'
    
    # 确认信息
    print("\n绑定信息确认:")
    print(f"  设备: {selected_device['dev_path']}")
    print(f"  绑定名称: {bind_name}")
    print(f"  权限: {permissions}")
    
    confirm = input("是否确认生成规则? (y/n): ").lower()
    if confirm != 'y' and confirm != 'yes':
        print("操作已取消")
        sys.exit(0)
    
    # 生成规则
    if generate_udev_rule(selected_device, bind_name, permissions):
        # 重新加载规则
        reload_udev_rules()
        print(f"\n操作完成! 设备已绑定到 /dev/{bind_name}")
        print(f"你可以使用以下命令验证: ls -l /dev/{bind_name}")

if __name__ == "__main__":
    # 检查是否以root权限运行
    if not os.geteuid() == 0:
        print("警告: 此脚本需要root权限才能写入udev规则文件。")
        print("请使用 sudo 运行: sudo python3 usb_udev_rule_generator.py")
        sys.exit(1)
    main()


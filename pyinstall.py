import subprocess
import sys
import os


def main():
    """打包项目为独立的 .exe 文件（包含所有 .txt 文件）"""
    try:
        # 确保当前目录是项目根目录（包含 main.py 和 txt 文件的目录）
        project_dir = os.path.dirname(os.path.abspath(__file__))
        os.chdir(project_dir)

        # 检查两个 txt 文件是否存在
        txt_files = ['accel_data.txt']
        missing_files = [f for f in txt_files if not os.path.exists(f)]

        if missing_files:
            print(f"\n❌ 错误：以下文件缺失，无法打包：{', '.join(missing_files)}")
            print("请确保这些文件在项目根目录中！")
            sys.exit(1)

        # 打包命令（关键：添加 --add-data 参数包含 txt 文件）
        command = [
            "pyinstaller",
            "--onefile",
            "--clean",
            "--windowed",
            # Windows 格式：--add-data "源文件;目标路径"
            "--add-data", "accel_data.txt;.",
            "main.py"
        ]

        print("开始打包... (请确保已安装 PyInstaller)")
        print(f"执行命令: {' '.join(command)}")

        # 调用 PyInstaller
        subprocess.run(
            command,
            check=True,
            shell=True  # Windows 需要 shell=True
        )

        print("\n✅ 打包成功！")
        print(f"生成的 .exe 位于: {os.path.join(project_dir, 'dist', 'main.exe')}")
        print("在任意 Windows 电脑上双击即可运行，无需安装 Python！")
        print("所有 .txt 文件已自动打包到 .exe 中")

    except subprocess.CalledProcessError as e:
        print(f"\n❌ 打包失败: {e}")
        print("请先安装 PyInstaller: pip install pyinstaller")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 未知错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
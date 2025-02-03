# ROS2.jl

ROS2 (ROS2 on Julia) is a Julia wrapper for ROS2's Python client library (rclpy). It provides a Julia-friendly interface for ROS2 functionality while leveraging the robust features of rclpy.

[日本語のREADMEはこちら](#rojjl-1)

## Features

- Topic Communication (Publisher/Subscriber)
- Service Communication (Client/Server)
- Action Communication (Client/Server)
- Timer Operations
- Parameter Management
- Logging Utilities

## Dependencies

### Required
- Julia 1.10+
- ROS2 (Jazzy)
- Python 3.12
- PyCall.jl
- ROS2 Python dependencies:
  - rclpy
  - std_msgs
  - example_interfaces

### Optional (Testing)
- Test.jl
- Aqua.jl

### Version Compatibility

| ROS2.jl Version | ROS2 Version | Julia Version | Python Version |
|----------------|--------------|---------------|----------------|
| 0.1.x          | Jazzy        | 1.10+        | 3.12          |

## Installation

1. Ensure you have ROS2 installed and sourced:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

2. Install ROS2.jl:
```julia
using Pkg
Pkg.add(url="https://github.com/your-username/ROS2.jl")
```

## Usage

### Basic Example

```julia
using ROS2

# Create a node
node = ROSNode("example_node")

# Create a publisher
pub = Publisher(node, "hello", "std_msgs.msg.String")

# Create and publish a message
msg = create_msg("std_msgs.msg.String")
msg.data = "Hello from Julia!"
publish(pub, msg)

# Create a subscriber
function callback(msg)
    println("Received: ", msg.data)
end
sub = Subscriber(node, "hello", "std_msgs.msg.String", callback)

# Spin
while is_ok()
    spin_once(node)
    sleep(0.1)
end
```

### Available Functions

- Node Management: `ROSNode`, `init`, `shutdown`, `spin`, `spin_once`, `is_ok`
- Topic Communication: `Publisher`, `Subscriber`, `create_msg`, `publish`
- Service Communication: `ServiceServer`, `ServiceClient`, `create_request`, `call`, `call_async`
- Action Communication: `ActionServer`, `ActionClient`, `send_goal`, `send_goal_sync`
- Timer Operations: `ROSTimer`, `cancel`, `reset`
- Parameter Management: `declare_parameter`, `get_parameter`, `set_parameter`
- Logging: `get_logger`, `debug`, `info`, `warn`, `error`, `fatal`

## Examples

Check the `examples/` directory for more detailed examples:
- Simple Publisher/Subscriber
- Service Client/Server
- Action Client/Server
- Parameter Usage
- Timer Operations

## Notes

1. Always source ROS2 environment before using ROS2:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

2. When using ROS2 in a ROS2 package:
   - Add ROS2 as a dependency in your package.xml
   - Configure your CMakeLists.txt to include Julia files
   - Run `colcon build` to build your package

3. ROS2 requires Python with rclpy installed and properly configured

## Contributing

We warmly welcome contributions to ROS2.jl! Whether it's bug fixes, new features, documentation improvements, or example code, your help is appreciated.

### Ways to Contribute

1. **Bug Reports and Feature Requests**
   - Use the GitHub Issues section
   - Provide detailed information about your environment
   - Include minimal reproducible examples

2. **Code Contributions**
   - Fork the repository
   - Create a new branch for your feature/fix
   - Write clear, documented code
   - Add tests for new functionality
   - Submit a Pull Request

3. **Documentation**
   - Improve or correct the documentation
   - Add examples
   - Translate documentation

4. **Examples**
   - Add new example code
   - Improve existing examples
   - Document real-world use cases

### Pull Request Process

1. Ensure your code follows the existing style
2. Add/update tests as needed
3. Update documentation as needed
4. Update CHANGELOG.md
5. Submit PR with clear description of changes

### Development Setup

```bash
# Clone the repository
git clone https://github.com/your-username/ROS2.jl
cd ROS2.jl

# Add the package in development mode
julia -e 'using Pkg; Pkg.develop(path=".")'

# Run tests
source /opt/ros/$ROS_DISTRO/setup.bash
julia --project -e 'using Pkg; Pkg.test()'
```

### Code Style Guidelines

1. Use meaningful variable names
2. Add comments for complex logic
3. Follow Julia style conventions:
   - Function names in snake_case
   - Type names in PascalCase
   - Module names in PascalCase
4. Document public functions with docstrings

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---

# ROS2.jl

ROS2 (ROS2 on Julia)は、ROS2のPythonクライアントライブラリ（rclpy）のJuliaラッパーです。rclpyの堅牢な機能を活用しながら、JuliaフレンドリーなインターフェースでROS2の機能を提供します。

## 機能

- トピック通信（パブリッシャー/サブスクライバー）
- サービス通信（クライアント/サーバー）
- アクション通信（クライアント/サーバー）
- タイマー操作
- パラメータ管理
- ロギングユーティリティ

## 依存関係

### 必須
- Julia 1.10以上
- ROS2 (Jazzy)
- Python 3.12
- PyCall.jl
- ROS2 Python依存関係：
  - rclpy
  - std_msgs
  - example_interfaces

### オプション（テスト用）
- Test.jl
- Aqua.jl

### バージョン互換性

| ROS2.jl バージョン | ROS2 バージョン | Julia バージョン | Python バージョン |
|------------------|----------------|-----------------|------------------|
| 0.1.x            | Jazzy          | 1.10以上        | 3.12            |

## インストール

1. ROS2がインストールされ、環境が設定されていることを確認：
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

2. ROS2.jlのインストール：
```julia
using Pkg
Pkg.add(url="https://github.com/your-username/ROS2.jl")
```

## 使用方法

### 基本例

```julia
using ROS2

# ノードの作成
node = ROSNode("example_node")

# パブリッシャーの作成
pub = Publisher(node, "hello", "std_msgs.msg.String")

# メッセージの作成と公開
msg = create_msg("std_msgs.msg.String")
msg.data = "Hello from Julia!"
publish(pub, msg)

# サブスクライバーの作成
function callback(msg)
    println("Received: ", msg.data)
end
sub = Subscriber(node, "hello", "std_msgs.msg.String", callback)

# スピン
while is_ok()
    spin_once(node)
    sleep(0.1)
end
```

### 利用可能な関数

- ノード管理: `ROSNode`, `init`, `shutdown`, `spin`, `spin_once`, `is_ok`
- トピック通信: `Publisher`, `Subscriber`, `create_msg`, `publish`
- サービス通信: `ServiceServer`, `ServiceClient`, `create_request`, `call`, `call_async`
- アクション通信: `ActionServer`, `ActionClient`, `send_goal`, `send_goal_sync`
- タイマー操作: `ROSTimer`, `cancel`, `reset`
- パラメータ管理: `declare_parameter`, `get_parameter`, `set_parameter`
- ロギング: `get_logger`, `debug`, `info`, `warn`, `error`, `fatal`

## サンプル

`examples/`ディレクトリに詳細な例があります：
- シンプルなパブリッシャー/サブスクライバー
- サービスクライアント/サーバー
- アクションクライアント/サーバー
- パラメータの使用
- タイマー操作

## 注意点

1. ROS2を使用する前に必ずROS2環境を読み込んでください：
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

2. ROS2パッケージでROS2を使用する場合：
   - package.xmlにROS2を依存関係として追加
   - CMakeLists.txtでJuliaファイルを含むように設定
   - `colcon build`でパッケージをビルド

3. ROS2には、rclpyがインストールされ、適切に設定されたPythonが必要です

## 貢献について

ROS2.jlへの貢献を歓迎します！バグ修正、新機能、ドキュメントの改善、サンプルコードなど、あらゆる形の協力を感謝します。

### 貢献の方法

1. **バグ報告と機能リクエスト**
   - GitHubのIssuesセクションを使用
   - 環境に関する詳細情報を提供
   - 最小限の再現可能な例を含める

2. **コード貢献**
   - リポジトリをフォーク
   - 機能/修正用の新しいブランチを作成
   - 明確で、ドキュメント化されたコードを作成
   - 新機能のテストを追加
   - プルリクエストを提出

3. **ドキュメント**
   - ドキュメントの改善や修正
   - 例の追加
   - ドキュメントの翻訳

4. **サンプル**
   - 新しいサンプルコードの追加
   - 既存のサンプルの改善
   - 実際の使用例のドキュメント化

### プルリクエストのプロセス

1. コードが既存のスタイルに従っていることを確認
2. 必要に応じてテストを追加/更新
3. 必要に応じてドキュメントを更新
4. CHANGELOG.mdを更新
5. 変更点を明確に説明してPRを提出

### 開発環境のセットアップ

```bash
# リポジトリをクローン
git clone https://github.com/your-username/ROS2.jl
cd ROS2.jl

# パッケージを開発モードで追加
julia -e 'using Pkg; Pkg.develop(path=".")'

# テストを実行
source /opt/ros/$ROS_DISTRO/setup.bash
julia --project -e 'using Pkg; Pkg.test()'
```

### コードスタイルガイドライン

1. 意味のある変数名を使用
2. 複雑なロジックにはコメントを追加
3. Juliaのスタイル規約に従う：
   - 関数名はsnake_case
   - 型名はPascalCase
   - モジュール名はPascalCase
4. 公開関数にはドキュメント文字列を追加

## ライセンス

このプロジェクトはMITライセンスの下で提供されています - 詳細はLICENSEファイルを参照してください。

© 2025 Michitoshi Tsubaki (@Michi-Tsubaki)

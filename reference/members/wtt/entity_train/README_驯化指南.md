# 代码"事物"模型驯化指南（无人机项目专用）

> 目标：让一个模型"认识"本仓库代码里的具体事物——**水桶（障碍物）、爆炸物（地雷）、
> 敌打击区（蓝面）、我方打击区、桥洞、绿线**——能自动判断"这段代码在干什么"，
> 甚至能回答/修改关于这些事物的问题。
>
> 硬件前提：RTX 4060 Laptop 8GB 显存（本指南所有参数按 8GB 给出）。

## 0. "驯化"的本质

模型驯化 = **数据 + 微调**。模型原本不认识"你们代码里的水桶"，因为通用模型里
"水桶"只是一只桶；你的代码里它是 `BUCKET`、直径 26cm、占一格禁入的障碍物。
驯化就是拿你的代码样本去调整模型参数，让它学到**你们项目里事物的代码语义**。

两条路线，按目标选：

| | 路线A：判别式分类器 | 路线B：LoRA 驯化代码大模型 |
|---|---|---|
| 能力 | 给一段代码 → 判它属于哪个事物 | 问答/解释/定位/改代码（生成式） |
| 模型 | CodeBERT(125M) / Qwen-Coder-0.5B | Qwen2.5-Coder-1.5B/3B-Instruct |
| 8GB 显存 | 轻松（2~4GB） | QLoRA 可行 |
| 数据量 | 数百~数千条片段+标签 | 500~2000 条高质量指令对 |
| 训练耗时 | 十几分钟 | 1~3 小时 |
| 本项目文件 | `entity_labels.py` `build_dataset.py` `train_classifier.py` `predict.py` | 见第 4 节（LLaMA-Factory） |

**建议顺序：先 A 后 B。** A 数据要求低、见效快，先把"识别事物"跑通；
B 在 A 的标签体系与数据基础上再升级成"能对话"。

## 1. 第一步永远是：给"事物"建标签体系

驯化前先想清楚"具体事物"有哪些、在代码里长什么样。本项目的建模结果
（`entity_labels.py`）：

| 标签 | 事物 | 代码判别特征（关键词/标识符） | 代表代码 |
|---|---|---|---|
| `bucket` | 水桶/散落障碍物 | `BUCKET` `BUCKETS_N` `OBSTACLE_D_CM` 水桶 障碍物 | `BUCKETS_N = 6` |
| `explosive` | 爆炸物/地雷 | `EXPLOSIVE(S)_N` `role=="explosive"` `MINE_ZONE` 地雷 排雷 | `explosive_order(...)` |
| `enemy_strike` | 敌打击区 | `ENEMY_N` `stop_face` `hit` 敌方 蓝面 激光打击 | `strike_cell(...)` |
| `friend_strike` | 我方打击区 | `FRIEND` 我方 无蓝面 | `kind = FRIEND` |
| `strike_zone` | 打击区通用/敌我判别 | `TARGET(S_N)` `strike_*` 打击区 | `target_rc()` |
| `bridge` | 桥洞 | `BRIDGE_CELLS` `BRIDGE_*` 桥洞 | `is_bridge(rc)` |
| `green_line` | 绿线禁入 | `GREEN_LINE` `LINE` 绿线 | `line_cells(...)` |
| `other` | 无关代码（**负类，必不可少**） | 无任何特征 | `def man(a,b): ...` |

要点：
- **负类决定模型的"否定能力"**——没有 `other`，模型会把所有代码硬归到某个事物；
- 标签允许层级/多标签（如 `enemy_strike` 是 `strike_zone` 的子类），先做单标签，
  跑通后再升级；
- 自动打标只是**初标**，训练前人工抽查 50~100 条改错，收益远大于加数据。

## 2. 路线A：判别式分类器（本目录已搭好，可直接跑）

流程：代码片段 → 冻结的代码编码器(CodeBERT) 转成向量 → 轻量分类头学"向量→事物"。

```bash
cd entity_train

# 0) 一次性安装依赖（国内网络）
pip install torch --index-url https://download.pytorch.org/whl/cu121
pip install transformers modelscope scikit-learn

# 1) 抽取标注样本（纯标准库，已跑通，产出 324 条）
python build_dataset.py --show 2          # 输出 dataset.jsonl

# 2) 训练（默认 AI-ModelScope/codebert-base，失败自动走 HF 镜像）
python train_classifier.py --head logreg

# 3) 推理
python predict.py --text "BUCKETS_N = 6   # 散落障碍物"
python predict.py --file ..\mission_planner.py --func strike_cell
python predict.py --file ..\mission_planner.py              # 逐函数摘要
```

**数据扩充与校标（决定上限，比调参重要）**：
1. 人工校标：打开 `dataset.jsonl` 抽查；标签错的改 `label` 字段；
2. 加样本：多几个种子/版本的 `mission_planner.py`、`car_client_demo.py`
   （`--files a.py b.py c.py`）、别人仓库里含 `explosive`/`bucket` 的代码；
3. 加负类：非本项目的通用代码（排序、网络请求等）→ `other`，防"见啥都是地雷"；
4. 平衡：小类（如 `bridge` 只有 8 条）用重复采样或 `class_weight`（已默认 balanced）。

**评估**：看 `output/results.txt` 的每类精确率/召回率，别只看总准确率；
混淆矩阵重点查 `enemy_strike ↔ friend_strike ↔ strike_zone` 三类的混淆。

### 2.5 训练完怎么导入（一行 import 即可用）

训练产物在 `output/`：`config.json`（配置）+ `head.joblib`（分类头）+
`results.txt`（评估报告）。注意：**训练只保存了分类头**，编码器本体（CodeBERT）
导入时会按 `config.json` 里的模型名自动从 ModelScope/HF 镜像重新加载一次
（约 500MB，只在第一次）。

```python
import sys
sys.path.insert(0, r"E:\1\无人机\entity_train")      # 换成你的实际路径
from entity_model import EntityClassifier

cls = EntityClassifier("output")                     # 指向训练产物目录
print(cls.classify("BUCKETS_N = 6"))
# [('bucket', 0.981), ('explosive', 0.012), ('other', 0.005)]

print(cls.classify_batch([code1, code2]))            # 批量
print(cls.tag("EXPLOSIVES_N = 8"))                   # 只取最优标签 -> explosive

# 集成进你的项目(例如给日志/新代码自动打事物标签):
def tag_entity(code: str) -> str:
    return cls.tag(code)
```

命令行等价写法：

```bash
python entity_model.py --model-dir output --text "EXPLOSIVES_N = 8"
python predict.py --model-dir output --file ..\mission_planner.py --func strike_cell
```

`EntityClassifier(model_dir, device=None)` 会自动用 GPU（无 GPU 回落 CPU），
可用 `device="cpu"` 强制。模型只需加载一次，之后 `classify`/`tag` 可反复调用。

### 2.6 如果你训的是 LoRA 大模型（路线B）

LoRA 产物是"增量权重"，不能直接 `AutoModel.from_pretrained` 导入，
先合并导出成完整模型再导入：

```bash
llamafactory-cli export \
  --model_name_or_path Qwen/Qwen2.5-Coder-1.5B-Instruct \
  --adapter_name_or_path ./entity_lora \
  --template qwen --finetuning_type lora \
  --export_dir ./entity_lora_merged --export_size 2 --export_legacy_format False
```

```python
from transformers import AutoModelForCausalLM, AutoTokenizer
model = AutoModelForCausalLM.from_pretrained("./entity_lora_merged",
                                             torch_dtype="auto", device_map="auto")
tok = AutoTokenizer.from_pretrained("./entity_lora_merged")
resp, _ = model.chat(tok, "敌打击区和我方打击区在代码里怎么区分？", history=None)
print(resp)
```

## 3. 路线B：LoRA 驯化代码大模型（生成式）

想让模型**解释**"为什么 `strike_cell` 要停蓝面正对格"、**定位**"所有排雷逻辑"、
**改代码**"把地雷区改成 4×5"，就要用这条。

### 3.1 数据：指令对（alpaca 格式，500~2000 条）

从你的 `manual_text.txt`、README、代码注释里提炼，每类事物写几十~几百条：

```json
[
  {"instruction": "这段代码处理的是哪个事物？解释它的作用。",
   "input": "def explosive_order(self, start, explosives, stop_cell):\n    ...",
   "output": "处理爆炸物(地雷)：对地雷区内的地雷做带转弯代价的 TSP 排序，输出踩雷顺序。"},
  {"instruction": "找出 mission_planner.py 里所有涉及水桶(障碍物)的配置。",
   "input": "",
   "output": "BUCKETS_N=6、OBSTACLE_D_CM=26.0、OBSTACLE_H_CM=25.0；水桶占一格禁入，路径规划时作不可通行格。"},
  {"instruction": "敌打击区和我方打击区在代码里怎么区分？",
   "input": "",
   "output": "有蓝面=敌方(ENEMY_N 个)，必须停靠蓝面正对格激光打击 3 次(role=stop_face/hit)；无蓝面=我方(FRIEND)，实体禁入、不打击。"}
]
```

高质量数据的三条铁律：**每条都要有"为什么"**（不只是标签，要有解释）；
**混入负例与刁钻问题**（"这段代码和水桶有关吗？" → "无关，它是坐标换算"）；
**留 10% 做考试集**，考试集在训练中绝对不碰。

### 3.2 训练（LLaMA-Factory，国内最省事）

```bash
pip install -e .          # 克隆 LLaMA-Factory 后
# 1) 从 ModelScope 拉模型
llamafactory-cli download --model-name Qwen/Qwen2.5-Coder-1.5B-Instruct

# 2) 配 QLoRA 参数（8GB 显存建议值，写进 examples/train_qlora/*.yaml）
llamafactory-cli train \
  --model_name_or_path Qwen/Qwen2.5-Coder-1.5B-Instruct \
  --template qwen \
  --dataset entity_qa \                  # 上面的 alpaca 数据注册为 entity_qa
  --finetuning_type lora \
  --quantization_bit 4 \
  --lora_rank 16 --lora_alpha 32 --lora_target all \
  --per_device_train_batch_size 2 --gradient_accumulation_steps 8 \
  --learning_rate 2e-4 --num_train_epochs 2.0 \
  --cutoff_len 1024 --fp16 --output_dir ./entity_lora
```

| 模型 | 8GB 显存可行性 | 建议 |
|---|---|---|
| Qwen2.5-Coder-1.5B-Instruct | 舒服（还有余量放长上下文） | **首选** |
| Qwen2.5-Coder-3B-Instruct | QLoRA 可行（batch=1~2） | 效果更好 |
| Qwen2.5-Coder-7B-Instruct | 勉强（batch=1、cutoff 512） | 不推荐新手 |

### 3.3 验收：考试集 + 双重检查

1. **考试集**：10% 留出数据考它，回答不要求一字不差，但事物、行为、参数必须对；
2. **防背题**：用**改写过的**问题（换个问法、改个参数）验证它真学会了概念；
3. **幻觉检查**：问"地雷直径是多少？"——README 说障碍物 26cm、地雷无直径，
   模型若瞎编一个数字就是没驯化好，回去补数据。

## 4. 常见坑（都在这类项目里实际发生过）

1. **只喂正样本**：没负类 → 模型对任何代码都硬分一个事物；
2. **类别严重不平衡**：小类（bridge/green_line）会被吞掉 → 加权/过采样；
3. **标签靠规则而非语义**：`hit` 在游戏代码里是"打击"，在别的库里是"命中缓存"，
   训练集要覆盖不同语义的同一词；
4. **测试集泄漏**：同一段代码既在训练集又在测试集 → 准确率虚高；
5. **过拟合**：样本几百条时别追求训练集 100%，看验证集；小数据用小学习率；
6. **生成式模型背答案**：问答对把原句塞进 output 背下来，换个问法就崩——考试集要改写问题。

## 5. 本目录文件

| 文件 | 作用 | 依赖 |
|---|---|---|
| `entity_labels.py` | 事物标签体系 + 规则打标 | 无（纯标准库） |
| `build_dataset.py` | 从代码抽取标注样本 → `dataset.jsonl` | 无 |
| `train_classifier.py` | 路线A训练（编码+分类头） | torch/transformers/modelscope/sklearn |
| `entity_model.py` | **导入接口**：`EntityClassifier` 一行 import 加载训练产物 | 同上 + joblib |
| `predict.py` | 路线A推理（判片段/函数/行，内部复用 entity_model） | 同上 |
| `dataset.jsonl` | 已生成的 324 条样本 | — |

**下一步建议**：① 装依赖跑 `train_classifier.py` 拿到路线A第一个模型；
② 抽 100 条样本人工校标提升质量；③ 满意后把同一份标签体系搬进路线B做 LoRA。

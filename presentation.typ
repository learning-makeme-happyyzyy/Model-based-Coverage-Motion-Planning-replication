// Some definitions presupposed by pandoc's typst output.
#let blockquote(body) = [
  #set text( size: 0.92em )
  #block(inset: (left: 1.5em, top: 0.2em, bottom: 0.2em))[#body]
]

#let horizontalrule = line(start: (25%,0%), end: (75%,0%))

#let endnote(num, contents) = [
  #stack(dir: ltr, spacing: 3pt, super[#num], contents)
]

#show terms: it => {
  it.children
    .map(child => [
      #strong[#child.term]
      #block(inset: (left: 1.5em, top: -0.4em))[#child.description]
      ])
    .join()
}

// Some quarto-specific definitions.

#show raw.where(block: true): set block(
    fill: luma(230),
    width: 100%,
    inset: 8pt,
    radius: 2pt
  )

#let block_with_new_content(old_block, new_content) = {
  let d = (:)
  let fields = old_block.fields()
  fields.remove("body")
  if fields.at("below", default: none) != none {
    // TODO: this is a hack because below is a "synthesized element"
    // according to the experts in the typst discord...
    fields.below = fields.below.abs
  }
  return block.with(..fields)(new_content)
}

#let empty(v) = {
  if type(v) == str {
    // two dollar signs here because we're technically inside
    // a Pandoc template :grimace:
    v.matches(regex("^\\s*$")).at(0, default: none) != none
  } else if type(v) == content {
    if v.at("text", default: none) != none {
      return empty(v.text)
    }
    for child in v.at("children", default: ()) {
      if not empty(child) {
        return false
      }
    }
    return true
  }

}

// Subfloats
// This is a technique that we adapted from https://github.com/tingerrr/subpar/
#let quartosubfloatcounter = counter("quartosubfloatcounter")

#let quarto_super(
  kind: str,
  caption: none,
  label: none,
  supplement: str,
  position: none,
  subrefnumbering: "1a",
  subcapnumbering: "(a)",
  body,
) = {
  context {
    let figcounter = counter(figure.where(kind: kind))
    let n-super = figcounter.get().first() + 1
    set figure.caption(position: position)
    [#figure(
      kind: kind,
      supplement: supplement,
      caption: caption,
      {
        show figure.where(kind: kind): set figure(numbering: _ => numbering(subrefnumbering, n-super, quartosubfloatcounter.get().first() + 1))
        show figure.where(kind: kind): set figure.caption(position: position)

        show figure: it => {
          let num = numbering(subcapnumbering, n-super, quartosubfloatcounter.get().first() + 1)
          show figure.caption: it => {
            num.slice(2) // I don't understand why the numbering contains output that it really shouldn't, but this fixes it shrug?
            [ ]
            it.body
          }

          quartosubfloatcounter.step()
          it
          counter(figure.where(kind: it.kind)).update(n => n - 1)
        }

        quartosubfloatcounter.update(0)
        body
      }
    )#label]
  }
}

// callout rendering
// this is a figure show rule because callouts are crossreferenceable
#show figure: it => {
  if type(it.kind) != str {
    return it
  }
  let kind_match = it.kind.matches(regex("^quarto-callout-(.*)")).at(0, default: none)
  if kind_match == none {
    return it
  }
  let kind = kind_match.captures.at(0, default: "other")
  kind = upper(kind.first()) + kind.slice(1)
  // now we pull apart the callout and reassemble it with the crossref name and counter

  // when we cleanup pandoc's emitted code to avoid spaces this will have to change
  let old_callout = it.body.children.at(1).body.children.at(1)
  let old_title_block = old_callout.body.children.at(0)
  let old_title = old_title_block.body.body.children.at(2)

  // TODO use custom separator if available
  let new_title = if empty(old_title) {
    [#kind #it.counter.display()]
  } else {
    [#kind #it.counter.display(): #old_title]
  }

  let new_title_block = block_with_new_content(
    old_title_block, 
    block_with_new_content(
      old_title_block.body, 
      old_title_block.body.body.children.at(0) +
      old_title_block.body.body.children.at(1) +
      new_title))

  block_with_new_content(old_callout,
    block(below: 0pt, new_title_block) +
    old_callout.body.children.at(1))
}

// 2023-10-09: #fa-icon("fa-info") is not working, so we'll eval "#fa-info()" instead
#let callout(body: [], title: "Callout", background_color: rgb("#dddddd"), icon: none, icon_color: black, body_background_color: white) = {
  block(
    breakable: false, 
    fill: background_color, 
    stroke: (paint: icon_color, thickness: 0.5pt, cap: "round"), 
    width: 100%, 
    radius: 2pt,
    block(
      inset: 1pt,
      width: 100%, 
      below: 0pt, 
      block(
        fill: background_color, 
        width: 100%, 
        inset: 8pt)[#text(icon_color, weight: 900)[#icon] #title]) +
      if(body != []){
        block(
          inset: 1pt, 
          width: 100%, 
          block(fill: body_background_color, width: 100%, inset: 8pt, body))
      }
    )
}



#let article(
  title: none,
  subtitle: none,
  authors: none,
  date: none,
  abstract: none,
  abstract-title: none,
  cols: 1,
  lang: "en",
  region: "US",
  font: "libertinus serif",
  fontsize: 11pt,
  title-size: 1.5em,
  subtitle-size: 1.25em,
  heading-family: "libertinus serif",
  heading-weight: "bold",
  heading-style: "normal",
  heading-color: black,
  heading-line-height: 0.65em,
  sectionnumbering: none,
  toc: false,
  toc_title: none,
  toc_depth: none,
  toc_indent: 1.5em,
  doc,
) = {
  set par(justify: true)
  set text(lang: lang,
           region: region,
           font: font,
           size: fontsize)
  set heading(numbering: sectionnumbering)
  if title != none {
    align(center)[#block(inset: 2em)[
      #set par(leading: heading-line-height)
      #if (heading-family != none or heading-weight != "bold" or heading-style != "normal"
           or heading-color != black) {
        set text(font: heading-family, weight: heading-weight, style: heading-style, fill: heading-color)
        text(size: title-size)[#title]
        if subtitle != none {
          parbreak()
          text(size: subtitle-size)[#subtitle]
        }
      } else {
        text(weight: "bold", size: title-size)[#title]
        if subtitle != none {
          parbreak()
          text(weight: "bold", size: subtitle-size)[#subtitle]
        }
      }
    ]]
  }

  if authors != none {
    let count = authors.len()
    let ncols = calc.min(count, 3)
    grid(
      columns: (1fr,) * ncols,
      row-gutter: 1.5em,
      ..authors.map(author =>
          align(center)[
            #author.name \
            #author.affiliation \
            #author.email
          ]
      )
    )
  }

  if date != none {
    align(center)[#block(inset: 1em)[
      #date
    ]]
  }

  if abstract != none {
    block(inset: 2em)[
    #text(weight: "semibold")[#abstract-title] #h(1em) #abstract
    ]
  }

  if toc {
    let title = if toc_title == none {
      auto
    } else {
      toc_title
    }
    block(above: 0em, below: 2em)[
    #outline(
      title: toc_title,
      depth: toc_depth,
      indent: toc_indent
    );
    ]
  }

  if cols == 1 {
    doc
  } else {
    columns(cols, doc)
  }
}

#set table(
  inset: 6pt,
  stroke: none
)

#set page(
  paper: "a4",
  margin: (x: 2.2cm,y: 2.2cm,),
  numbering: "1",
)

#show: doc => article(
  title: [Paper 05 复现方案],
  subtitle: [Week 1 Presentation],
  authors: (
    ( name: [Zhouyiyang Yang],
      affiliation: [],
      email: [] ),
    ),
  font: ("Songti SC",),
  fontsize: 11pt,
  sectionnumbering: "1.1.a",
  toc: true,
  toc_title: [Table of contents],
  toc_depth: 2,
  cols: 1,
  doc,
)

// 字体配置（raw Typst 注入，覆盖 Quarto 默认值）：
// 正文：拉丁 Libertinus Serif，中文 Songti SC（宋体），其余符号回退 PingFang SC
#set text(font: ("Libertinus Serif", "Songti SC", "PingFang SC"))
// 章节标题：PingFang SC
#show heading: set text(font: ("PingFang SC"))
// 代码块：Menlo，中文/制表符回退 PingFang SC
#show raw: set text(font: ("Menlo", "PingFang SC"))
// 数学模式：符号用 New Computer Modern Math，数学中的普通文本
// （\text{...}、upright(...)）中文回退到 Songti SC
#show math.equation: set text(font: ("New Computer Modern Math", "Songti SC"))
老师，我这段时间先按照 Paper 05 的整体流程，把传统的工业机器人 3D 表面检测规划流程拆成几个模块，目前已经完成了第一个模块 M1：#strong[Surface Discretization，也就是表面离散化];。

我目前没有直接把整个算法一次性实现，而是按照模块逐步复现，这样后面如果结果有问题，可以定位到底是表面离散、可见性、路径规划还是优化算法的问题。

Paper 05 的整体流程，我目前理解为：

#strong[工件 Mesh → 表面离散 → 候选观察位姿 → 可见性计算 → 观察点选择与排序 → 机器人运动执行。]

所以 M1 是整个后续规划的输入基础。

#horizontalrule

== 二、M1 要解决什么问题
<二m1-要解决什么问题>
原始 CAD / Mesh 表示的是一个连续的三维表面。

但是后面的算法需要回答：

#quote(block: true)[
"哪些表面已经被检测到了？"
]

所以首先需要把连续表面离散成有限数量的 surface patches。

我目前把它表示成：

$ S = { s_1 \, s_2 \, dots.h \, s_m } $

每个 patch 至少包含：

- patch ID
- vertices
- center
- normal
- area

这样到了后面的 M2、M3，就可以以 patch 为单位讨论：

#quote(block: true)[
从哪个 viewpoint 可以看到哪个 patch。
]

#horizontalrule

== 三、M1 的具体实现
<三m1-的具体实现>
我目前的 M1 流程是：

#strong[Mesh loading]

→ 去除退化三角形

→ 修正/检查法向

→ 居中与尺度处理

→ 均匀表面重采样

→ 得到目标数量附近的三角 surface patches

→ 计算每个 patch 的 center、normal、area

→ 保存为 NPZ + JSON

同时增加了 visualization 和单元测试。

这里需要特别说明：

Paper 05 本身并没有明确规定具体的 surface discretization 方法，也没有明确给出 m=1000 这个参数。

所以我现在采用的：

#quote(block: true)[
均匀重采样 + 目标约 1000 个 patches
]

是#strong[复现方案中的自行设定];，不是论文原始参数。

我觉得这个区别比较重要，所以在代码和复现记录里面把它单独标出来了。

#horizontalrule

== 四、M1 的验证结果
<四m1-的验证结果>
我用一个自己构造的 L 型支架作为最小测试工件。

原始 Mesh：

- 12 个 vertices
- 20 个 triangles
- watertight
- 总表面积约 0.1024 m²

经过 M1 后得到：

- 976 个 surface patches
- 目标数量为 1000
- 误差约 -2.4%，在预先设定的 ±5% 范围内

另外检查了几个比较重要的量。

首先是面积：

#quote(block: true)[
所有 patch 的面积之和与原始 mesh 面积基本一致，偏差约为 0。
]

说明离散过程中没有明显的面积损失。

其次是法向：

#quote(block: true)[
976/976 个 patch 的法向通过外法向检查。
]

每个 normal 的长度也都约等于 1。

另外还检查了 patch 尺寸的均匀性，当前这个 bracket 工件的 std/mean 约为 0.328。

#horizontalrule

== 五、我目前认为 M1 最重要的输出
<五我目前认为-m1-最重要的输出>
我觉得 M1 最核心的不是“最终得到了 976 还是 1000 个 patch”，而是建立了一个稳定的数据接口：

```text
Mesh
  ↓
M1
  ↓
Surface Patches
  ↓
{
    center,
    normal,
    area,
    vertices
}
```

后面的模块就可以直接基于这个离散表面工作。

例如 M2 要解决：

#quote(block: true)[
对这些 surface patches，应该在哪里生成 candidate viewpoints？
]

M3 则进一步解决：

#quote(block: true)[
每个 candidate viewpoint 实际能够看到哪些 surface patches？
]

最终才形成 visibility matrix。

所以 M1 实际上是把后面整个 coverage planning 问题的“目标空间”离散化了。

#horizontalrule

== 六、目前发现的论文复现问题
<六目前发现的论文复现问题>
在复现 M1 的过程中，我发现 Paper 05 对这一部分描述其实比较简略。

论文只明确使用了：

#quote(block: true)[
surface patch set S containing m triangular patches
]

但没有详细说明：

- surface patches 是如何生成的；
- m 应该取多少；
- patch 的大小如何确定。

因此这一部分目前不能声称是“严格复现论文原始实现”。

我现在采取的是：

#quote(block: true)[
明确区分论文给出的内容和复现者自行设计的内容。
]

这样后面写实验结果的时候，可以准确说明哪些是 paper setting，哪些是 reproduction setting。

#horizontalrule

== 七、目前 M1 的结论
<七目前-m1-的结论>
所以目前 M1 我认为已经可以作为后续模块的输入。

它完成了三个事情：

#strong[第一，完成了连续 Mesh → 离散 Surface Patches 的转换。]

#strong[第二，验证了面积、法向、patch 数量以及基本几何质量。]

#strong[第三，建立了后续 M2/M3 可以直接使用的数据接口。]

目前 M1 测试：

```text
25 passed
```

没有进入 M2 的实现。

#horizontalrule

== 八、下一步
<八下一步>
下一步我准备进入 M2：#strong[Candidate Viewpoint Generation];。

也就是在已经得到 surface patches 的基础上，生成机器人/传感器可能的观察位置和观察方向。

这一阶段我会重点确认 Paper 05 中 candidate viewpoint generation 和 potential-field viewing direction 的定义，然后再进入 M3 的 visibility calculation。

M3 会比较关键，因为最终需要得到：

$ A in { 0 \, 1 }^(N times M) $

其中：

$ A_(i j) = 1 $

表示 candidate viewpoint (i) 能够看到 surface patch (j)。

我准备先把 M1、M2、M3 这几个几何模块跑通，再进入后面的 viewpoint selection 和 path optimization。

老师，我目前先按照 Paper 05 的流程做模块化复现，现在已经完成第一个模块 M1，也就是 Surface Discretization。

M1 的作用是把原始 CAD/Mesh 的连续表面离散成有限数量的 triangular surface patches，因为后面的 viewpoint generation 和 visibility calculation 都需要以 patch 为基本单位。

目前我的流程是：

#strong[Mesh → 预处理 → 均匀重采样 → Surface Patches]

每个 patch 保存 center、normal、area 和 vertices 等信息。

我用一个 L 型 bracket 做了最小验证。原始 mesh 有 20 个三角形、总面积 0.1024 平方米，经过 M1 后得到 976 个 patches，目标是约 1000 个，在设定的 ±5% 范围内。所有 patch 的面积总和与原始 mesh 基本一致，976 个 patch 的外法向检查也全部通过。目前测试是 25 个全部通过。

这里我还发现一个比较重要的问题：Paper 05 本身没有明确说明 surface discretization 的具体方法，也没有给出 m 的具体取值。所以现在的“均匀重采样、约 1000 个 patches”属于我的 reproduction setting，而不是论文原始参数，我已经把这一点单独记录下来。

下一步我会基于这些 surface patches 做 M2，也就是 Candidate Viewpoint Generation，然后再做 M3 的 visibility matrix。

#horizontalrule

一、先说一下 M2 在整个流程中的位置

我现在对 Paper 05 整个流程的理解是：

Mesh

→ M1 Surface Discretization

→ M2 Candidate Viewpoint Generation

→ M3 Visibility Calculation

→ M4 Travel Cost

→ M5 Viewpoint Selection / Ordering。

所以 M1 和 M2 解决的问题其实是不一样的：

M1 是把：

"我要检测哪些表面？"

离散成 surface patches。

M2 则是：

"我可以从哪里去看这些表面？"

生成 candidate viewpoints。

而到了 M3，才真正判断：

"从某一个 viewpoint，到底能看到哪些 patch？"

也就是最后建立 visibility matrix。

所以我目前特意把 M2 和 M3 分开，没有在 M2 里面提前做遮挡或者 visibility filtering。

二、M2 具体做了什么

M2 的输入就是 M1 得到的 surface patches。

每个 patch 有：

center normal area vertices。

然后我按照 Paper 05 和复现指南的定义，在工件周围生成一批 candidate viewpoints。

每个 viewpoint 主要包含：

position viewing direction orientation。

这里的位置不是简单地直接在每个 patch 法向外面放一个点，而是按照论文中描述的随机采样方式，加上我们复现方案中对采样范围和机器人可达性的简化。

最终得到：

\[ V={v\_1,v\_2,,v\_N}\]

目前支持论文里面的：

\[ N=200,~300,~400\]

三个 candidate 数量。

三、位置是怎么生成的

这里我做了一个区分。

Paper 05 原文对这一部分给出的描述比较概括，大致是：

在工件周围的 Euclidean space 中，在 sensor maximum viewing range 内进行冗余随机采样。

但是论文没有把具体的随机采样分布完全展开。

所以我按照复现指南具体化成了一个球壳采样方案：

首先围绕工件的 bounding box 中心进行采样，然后根据 FOD 范围确定距离，同时限制在桌面以上。

另外，论文原始方法还包含机器人 IK 和 FCL 的可达性、碰撞过滤。

由于我们目前没有真实 ABB 机器人和完整的 ROS/IK/FCL 环境，所以这里没有直接复现这一套，而是按照复现指南使用一个简化的 reachable shell 来替代。

这个地方我已经明确记录成：

reproduction simplification

而不是说这是论文原始实现。

四、M2 中比较关键的是 viewing direction

这部分我花了一些时间核对 Paper 05 的 Eq. (10)。

论文中是通过一个 potential-field 的形式，根据 candidate viewpoint 周围一定范围内的 surface patches，计算一个加权方向。

直观上可以理解成：

```
         多个 surface patches
              ↙ ↓ ↘
               \ | /
                \|/
             viewpoint
                 ●
                 ↓
          viewing direction
```

也就是说，一个 viewpoint 周围如果有多个可以采集的 surface patches，那么它的 viewing direction 会综合这些 patch 的方向，而不是简单地只指向某一个 patch。

我这里还发现了一个论文表述上的符号问题。

论文公式中 (d\_i) 的文字定义和后面“视点朝向可采集面片加权平均”的描述，在相机光轴方向的理解上存在一个正负号上的歧义。

所以我没有直接把公式机械地照抄，而是结合论文对光轴方向的文字描述采用了相应的方向解释，并且已经在代码和 README 中把这个解释记录下来。

我把它标记成：

paper ambiguity / reproduction interpretation

而不是论文明确规定的参数。

五、目前 M2 的结果

在 bracket 这个测试工件上，我用 seed=0 生成了 400 个 candidate viewpoints。

然后 200 和 300 使用这个 400 个 candidate set 的前缀。

这样做是因为 Paper 05 明确提到：

三种不同方法使用相同的 sampled candidate viewpoints。

所以后面做实验比较的时候，我希望保证不同方法面对的是完全相同的 candidate set，而不是每次重新随机采样。

目前：

N=200：200 个 N=300：300 个 N=400：400 个

都正常生成。

M1 的 25 个测试也仍然全部通过，加上 M2 的 19 个测试，目前一共：

\[ 44\]

六、我做的几个几何 sanity check

目前比较重要的结果是：

400 个 viewpoint 都满足当前设定的中心采样壳和 reachable shell。

viewing direction 都完成了单位化，最大归一化误差大约：

\[ 2.2^{-16}\]

没有发现 viewpoint 落到工件内部。

另外我也检查了 viewing direction 和工件中心以及最近 surface patch 的关系，目前都满足预期的朝向关系。

可视化上，我也把：

工件 + 400 个 viewpoints + viewing direction arrows

放在了一张图里面。

这样可以直接人工检查这些 candidate viewpoints 是否分布在合理的观察区域。

七、这里有一个比较重要的问题：d\_max

M2 里面目前有一个我觉得需要特别说明的问题。

Paper 05 定义了一个 (d\_{})，也就是 Eq. (10) 中参与 potential-field 计算的最大 inclusion distance。

但是论文没有给出明确的数值。

我们的复现指南初始设置是：

\[ d\_{}=0.30m\]

但是按照前面的采样壳设置，最小的 viewpoint-to-surface 距离大约已经是：

\[ FOD\_{min}+margin=0.40+0.05=0.45m\]

所以如果直接使用 0.30m，实际上 viewpoint 周围很容易没有满足条件的 patch。

这会导致 Eq. (10) 的邻域为空。

所以我按照复现指南里的诊断方法对 (d\_{}) 做了调参。

在当前 bracket 工件上，最后得到：

\[ d\_{}0.746m\]

使得邻域 patch 数的中位数达到 10。

因此这里的 0.746m 不是论文参数，而是当前复现设置下根据指南得到的 reproduction parameter。

八、还有一个我觉得值得注意的现象

即使 viewpoint 是在一个比较大的中心球壳上采样的，也不是所有 viewpoint 到真实表面的距离都会严格落在 FOD 0.4--1.0m 内。

当前 bracket 上：

大约 93.8% 的 viewpoints 到最近 patch 的距离落在 FOD 范围内。

还有大约 6.2% 超过 1.0m。

这个我目前没有在 M2 里面强行删除，因为我理解：

M2 的职责是生成 candidate viewpoints，真正的 FOD、FOV、viewing angle 以及 occlusion visibility filtering 应该在 M3 统一判断。

所以我准备把这部分交给 M3。

九、目前 M2 的结论

所以目前 M2 我认为已经可以作为 M3 的输入。

现在整个流程已经变成：

Mesh ↓ M1 ↓ Surface Patches S ↓ M2 ↓ Candidate Viewpoints V ↓ M3 ↓ Visibility Matrix A

其中：

\[ S={s\_1,,s\_M}\]

而：

\[ V={v\_1,,v\_N}\]

下一步 M3 就是要建立：

\[ A^{NM}\]

其中：

\[ A\_{ij}=1\]

表示第 (i) 个 candidate viewpoint 可以看到第 (j) 个 surface patch。

M3 我觉得会是后面比较关键的一步，因为需要同时处理：

FOV FOD viewing angle occlusion / ray casting

尤其是要区分：

射线首先命中了目标 patch

和：

目标 patch 前面还有其他几何体遮挡。

所以我下一步准备先把 Paper 05 中 viewing angle ≤75° 的具体定义再核对清楚，然后开始实现 M3。 老师好，我这两天主要完成了复现流程里的 #strong[M3------Visibility，也就是可见性判断];。

前面的 M1 和 M2 分别解决了两个问题：

- M1：把工件表面离散成大约 1000 个 triangular patches，也就是确定“#strong[要检测什么];”；
- M2：在工件周围生成 200、300、400 个候选 viewpoints，也就是确定“#strong[从哪里看];”。

所以 M3 要解决的问题就是：

#quote(block: true)[
#strong[对于每一个 viewpoint，它到底能够看到哪些 surface patches？]
]

最终得到一个可见性矩阵：

$ A in { 0 \, 1 }^(N times M) $

其中 (N) 是 viewpoint 数量，(M=976) 是当前 bracket 的 surface patches。

如果

$ A_(i j) = 1 $

就表示第 (i) 个 viewpoint 可以完整看到第 (j) 个 patch。

#horizontalrule

== 1. M3 的核心判据
<m3-的核心判据>
我按照论文原文实现了 patch 的可见性判定。

论文明确要求：

#quote(block: true)[
一个 triangular patch 可见，需要它的#strong[所有三个顶点];满足相应的可见性条件。
]

因此整体上可以理解成：

$ A_(i j) = 1 $

当且仅当这个 patch 同时满足：

+ #strong[FOV]
+ #strong[FOD]
+ #strong[Viewing angle]
+ #strong[Occlusion]

其中前三个主要是在判断“这个面片是否处于合适的观察范围和角度”，最后一个判断“中间有没有其他几何体挡住”。

具体来说：

#strong[FOD] 使用论文 Table I 的参数：

$ 0.40 upright(" m") lt.eq parallel q - p_c parallel lt.eq 1.00 upright(" m") $

而且是对 patch 的三个顶点分别检查。

#strong[FOV] 使用论文给出的：

$ 30^circle.stroked.tiny times 21^circle.stroked.tiny $

也就是水平 ±15°、垂直 ±10.5°。

我把顶点变换到 M2 定义的 camera frame 中，然后判断：

$ lr(|x|) \/ z lt.eq tan 15^circle.stroked.tiny $

以及

$ lr(|y|) \/ z lt.eq tan 10.5^circle.stroked.tiny $

同时要求 (z\>0)。

#horizontalrule

== 2. 这里有一个论文中的 ambiguity
<这里有一个论文中的-ambiguity>
比较需要说明的是 #strong[75° viewing angle];。

论文给了 75°，但是正文没有非常明确地定义这个角度究竟是哪两个向量之间的夹角。

所以我没有把这个直接当成论文明确给出的数学公式，而是结合复现指南里的 toy example 来确定解释。

现在使用的是：

$ theta = arccos frac((p_c - c_j) dot.op n_j, parallel p_c - c_j parallel) $

也就是：

#quote(block: true)[
#strong[视点指向 patch center 的方向，与 patch 外法向之间的夹角。]
]

要求：

$ theta lt.eq 75^circle.stroked.tiny $

这样正对面就是 0°，而超过 75° 的 grazing view 会被过滤掉。

另外，我还发现指南里面原来的代码片段有一个符号问题：如果直接使用其中的 (c\_j-p\_c)，正对面会得到 180°，和它自己的测试例冲突。

所以代码采用了和测试例一致的方向，并且在 README 里把这个问题明确记录成 #strong[interpretation，而不是论文原文事实];。

#horizontalrule

== 3. 遮挡是 M3 里面最关键的一部分
<遮挡是-m3-里面最关键的一部分>
对于每个 patch 的三个顶点，我都从 viewpoint 向这个顶点发射 ray。

这里没有简单地做：

#quote(block: true)[
"ray hit 到东西 → 就认为被遮挡"。
]

而是区分：

- 命中目标 patch 本身；
- 命中目标之前先撞到了其他 triangle。

只有第二种情况才认为存在 occlusion。

另外，由于射线最终本身就是射向目标顶点，所以会存在 self-intersection 的数值问题。

这里使用：

$ epsilon.alt = 10^(- 3) upright(" m") $

作为 self-hit tolerance。

这个参数属于复现/工程处理，而不是论文 Table I 中的参数。

我专门写了 blocker/target、self-hit、epsilon 等 toy tests 来验证这一点。

#horizontalrule

== 4. 测试结果
<测试结果>
目前 M1、M2、M3 一共：

$ 25 + 19 + 22 = 66 $

个测试全部通过。

其中 M3 自己有 22 个测试，包括：

- FOV；
- FOD；
- viewing angle；
- FOV boundary；
- 三顶点聚合；
- occlusion；
- target self-hit；
- epsilon；
- camera frame；
- bracket integration 等。

所以目前 M3 的 PASS 不是只跑了一遍 bracket，而是先用 toy geometry 验证各个判据，再进行完整 integration。

#horizontalrule

== 5. Bracket 的实际结果
<bracket-的实际结果>
在当前 bracket 上，M1 得到：

$ M = 976 $

个 patches。

然后分别使用：

$ N = 200 \, med 300 \, med 400 $

个 viewpoints。

得到：

#table(
  columns: 3,
  align: (right,right,right,),
  table.header([Viewpoints], [Visibility Matrix], [Union Coverage],),
  table.hline(),
  [200], [(200)], [70.1%],
  [300], [(300)], [83.1%],
  [400], [(400)], [83.1%],
)
其中 400 个 viewpoint 下，单个 viewpoint 最多可以看到 556 个 patches，也就是大约：

$ 57.0 % $

#horizontalrule

== 6. 这里发现了一个比较重要的问题
<这里发现了一个比较重要的问题>
400 个 viewpoints 最终 union coverage 只有 #strong[83.1%];，乍看起来可能像 M3 有问题。

但是我进一步检查了 zero-column。

976 个 patches 中有 165 个完全没有被任何 viewpoint 看到。

其中：

$ 164 $

个恰好是 bracket 的#strong[整个底面];。

进一步检查发现，这是当前 M2 的几何约束导致的结构性结果：

#quote(block: true)[
viewpoint 被限制在台面以上，而 bracket 底面位于台面以下，所以从当前 viewpoint sampling strategy 下，底面基本不可能满足合理的 viewing angle。
]

因此这不是简单的 ray-casting bug，我目前#strong[没有为了提高 coverage 去修改 M2 或 M3];。

剩下还有 1 个属于侧面的 grazing patch。

所以现在比较重要的是：

#quote(block: true)[
在 M5 进行 coverage optimization 之前，需要明确论文里 coverage 的 denominator 怎么定义。
]

一种方式是把接触台面的底面也计入分母，那么当前 bracket 的理论 coverage 上限就大约在 83%；

另一种是把物理上不可检测的底面作为 inspection mask 排除。

我目前没有擅自选择，而是先把这个问题保留下来。

#horizontalrule

== 7. 另外一个现象
<另外一个现象>
n=200 时还有比较明显的 grazing-angle patches 没有被覆盖：

- n=200：292 个 zero-columns；
- n=300：165 个；
- n=400：165 个。

也就是说从 200 增加到 300 后，coverage 从 70.1% 提升到 83.1%，而 300 到 400 没有继续提升。

这说明当前候选 viewpoint 数量增加到 300 左右之后，主要剩下的就不是随机 sampling density 问题，而是前面说的#strong[结构性不可见区域];。

这个结果对后面的 M5 也比较有意义。

#horizontalrule

== 8. 目前阶段结论
<目前阶段结论>
所以目前我认为 M3 可以暂时 PASS。

现在整个 pipeline 已经形成：

$ #box(stroke: black, inset: 3pt, [$ S arrow.r V arrow.r A $]) $

也就是：

$ upright("Surface Patches") arrow.r upright("Candidate Viewpoints") arrow.r upright("Visibility Matrix") $

到这里，我们已经知道：

#quote(block: true)[
#strong[每个候选 viewpoint 能看到哪些 surface patches。]
]

下一步进入 M4。

M4 的任务就不再是判断“能不能看到”，而是给 viewpoint 之间建立 #strong[travel cost];，也就是：

#quote(block: true)[
#strong[从一个 viewpoint 移动到另一个 viewpoint 需要多大代价。]
]

目前计划按照复现指南，先使用：

$ C_(i j) = alpha thin d_(i j) + beta thin theta_(i j) $

其中：

$ alpha = 1.2 upright(" s/m") \, #h(2em) beta = 0.4 upright(" s/rad") $

并加入 home pose 的往返成本。

这样 M4 完成以后，M5 就可以真正开始做：

#quote(block: true)[
#strong[在 coverage 和 motion cost 之间选择一组 viewpoint，并确定访问顺序。]
]

所以目前我先停在 M3，没有提前进入 M4。 ① 一张 pipeline 图

M1 Surface ↓ 976 patches ↓ M2 Candidate Viewpoints ↓ 200 / 300 / 400 viewpoints ↓ M3 Visibility ↓ A ∈ {0,1}^{N×976} ↓ M4 Travel Cost ↓ M5 Viewpoint Selection

② A\_bracket\_400.png

重点指着四个区域解释：

"这里是 visibility matrix，这里是每个 patch 被多少 viewpoint 看到，这里是某一个 viewpoint 能看到的区域，这里反过来显示某个 patch 被哪些 viewpoint 看到。"

这张图比讲 20 个代码文件有价值很多。

③ 结果表

N A.shape Union Coverage 200 200×976 70.1% 300 300×976 83.1% 400 400×976 83.1%

然后重点说一句：

"83.1% 不是简单的算法失败，我追踪 zero-column 后发现，其中 164 个是整个底面，因为当前 viewpoint sampling 位于台面以上，因此底面属于结构性不可见区域。"

这个其实是你这次 M3 汇报里最值得讲的发现。

老师如果追问，你重点准备这 5 个问题

Q1：为什么 patch 要三个顶点全部满足？

答：

因为论文原文明确写的是 for all of its vertices，所以不是中心点可见就算整个 triangle 可见。

Q2：为什么 83.1%，是不是 M3 有 bug？

答：

我检查了 zero-column。165 个完全不可见 patch 中，164 个是整个底面，和 M2 的台面约束及 viewing-angle constraint 一致，因此目前判断主要是结构性不可见，而不是 ray-casting 错误。

Q3：为什么 400 比 300 没提高？

答：

200→300 主要解决了 sampling density 导致的 grazing patches；300→400 后剩余不可见区域主要是结构性不可见，所以继续增加 viewpoint 数量没有明显收益。

Q4：75° 是论文明确的吗？

这里一定不要说“论文明确规定了这个公式”。

说：

75° 数值来自论文，但具体夹角定义正文没有完全展开。我结合复现指南的 toy example 采用了 viewpoint-to-patch-center 与 patch outward normal 的夹角，并把它明确标记为 reproduction interpretation。

这个表述很重要，因为它体现你在区分 paper fact 和 reproduction assumption。

Q5：为什么不直接把底面删掉？

答：

因为这会改变 coverage denominator，从而影响后面的 M5 fitness。我目前先不修改 M1/M2/M3，准备在 M5 开始前明确 inspection mask / coverage denominator 的定义，并报告两种口径。

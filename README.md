# 1. gst-customvenc-plugin : 编写自己的gstreamer插件

本文档记录了自己学习写gstreamer plugin的过程.

# 2. 步骤(适合自己理解的一般过程)
注: 仅参考gstreamer的官方文档来编写插件是有难度的, 个人觉得官方文档不是很友好, 建议直接去参考gstreamer自带的插件, 如gst-plugin-good, gst-plugin-base, gst-plugin-bad等, 毕竟先跑起来才更容易理解吧.

1.调用GST_PLUGIN_DEFINE(major,minor,name,description,init,version,license,package,origin)宏设置版本号, 插件名称, 初始化函数, 许可证以及插件描述等信息.

```c
static gboolean
plugin_init (GstPlugin * customvenc){

    // 方便用户层进行debug
    GST_DEBUG_CATEGORY_INIT (gst_customvenc_debug, "customvenc", 0,
        "customized video encoding element");

    return gst_element_register (customvenc, "customvenc",
        GST_RANK_PRIMARY, GST_TYPE_CUSTOMVENC);
}

GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR, GST_VERSION_MINOR, customvenc,
    "customized video encoder plugins", plugin_init,
    VERSION, "LGPL", "customized video ecoding",
    "Local"
)
```
2.完成 xxx_class_init()函数, 根据具体功能实现必要的callback函数(gstreamer框架调用)以及相关property参数的添加(有些参数需要用户在外部修改设置)
```c
static void
gst_customvenc_class_init (GstCustomEncClass * klass)
{
    GObjectClass *obj_class;
    GstElementClass *ele_class;
    GstVideoEncoderClass *enc_class;
    GstPadTemplate *sink_templ;
    GstCaps *supported_sinkcaps;

    obj_class = G_OBJECT_CLASS (klass);
    ele_class = GST_ELEMENT_CLASS (klass);
    enc_class = GST_VIDEO_ENCODER_CLASS (klass);

	// callback相关
    obj_class->set_property = gst_customvenc_set_property;
    obj_class->get_property = gst_customvenc_get_property;
    obj_class->finalize = gst_customvenc_finalize;

	......
	// property相关
    g_obj_class_install_property (obj_class, P_GOP,
        g_param_spec_int ("gop", "GOP", "key frame interval",
            -1, 1000, P_IDR_PERIOD_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
            
	......

    gst_ele_class_set_static_metadata (
          ele_class, "Customized Video Encoder", "Video Encoder",
          "Customized Video Encoder Plugin", "HenryLee <henry_1_lee@163.com>");

	// 支持的从上游传下来的stream的能力(capability)实例
    supported_sinkcaps = gst_caps_new_simple ("video/x-raw",
        "framerate", GST_TYPE_FRACTION_RANGE, 0, 1, G_MAXINT, 1,
        "width", GST_TYPE_INT_RANGE, 16, G_MAXINT,
        "height", GST_TYPE_INT_RANGE, 16, G_MAXINT, NULL);

	// 添加sinkcaps实例至template
    sink_templ = gst_pad_template_new ("sink",
        GST_PAD_SINK, GST_PAD_ALWAYS, supported_sinkcaps);

    gst_caps_unref (supported_sinkcaps);

    gst_ele_class_add_pad_template (element_class, sink_templ);
    gst_ele_class_add_static_pad_template (element_class, &source_pad);
}
```
3.分别实现上面确定的callback函数, 见下表(具体实现参考代码)

| gstreamer框架的callback与函数实现的映射 | class类型 |  描述    |
| ------------------------------------------------------ | ---- | ---- |
| set_property  <==>  gst_customvenc_set_property | G_OBJECT_CLASS | 属性设置, 实现此函数, 用户可以在外部修改相关属性值 |
| get_property <==>  gst_customvenc_get_property | G_OBJECT_CLASS | 属性获取, 实现此函数, 用户可以在外部获取当前相关属性值 |
| finalize  <==>  gst_customvenc_finalize | G_OBJECT_CLASS | 类似析构函数, 将申请的资源进行释放回收                 |
| start  <==>  gst_customvenc_start | GST_VIDEO_ENCODER_CLASS | 当开始处理pipeline上的stream流时, 该函数会被执行 |
| stop  <==>  gst_customvenc_stop | GST_VIDEO_ENCODER_CLASS | 当pipeline上无待处理的stream流时, 该函数会被执行 |
| flush  <==>  gst_customvenc_flush | GST_VIDEO_ENCODER_CLASS | 刷新流, 刷新流时我们回去重新初始化encoder |
| set_format  <==>  gst_customvenc_set_format | GST_VIDEO_ENCODER_CLASS | 格式设置, 只要插件的pad属性被修改，gstreamer就会调用它 |
| sink_query  <==>  gst_customvenc_sink_query | GST_VIDEO_ENCODER_CLASS | 查询pad支持的caps, 上下游协商 |
| handle_frame  <==>  gst_customvenc_handle_frame | GST_VIDEO_ENCODER_CLASS | 每来一帧都会调用此函数进行处理(这里实现主要逻辑) |


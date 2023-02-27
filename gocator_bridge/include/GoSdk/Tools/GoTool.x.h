/** 
 * @file    GoTool.x.h
 *
 * @internal
 * Copyright (C) 2016-2020 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TOOL_X_H
#define GO_TOOL_X_H

#include <kApi/Data/kXml.h>
#include <GoSdk/Tools/GoMeasurement.h>
#include <GoSdk/Tools/GoFeature.h>

typedef k32s GoToolFormat;
#define GO_TOOL_FORMAT_UNKNOWN                      (-1)
#define GO_TOOL_FORMAT_STANDARD                     (0)
#define GO_TOOL_FORMAT_GDK_USER                     (1) 
#define GO_TOOL_FORMAT_GDK_INTERNAL                 (2)

typedef struct GoToolVTable
{    
    kObjectVTable base; 
    kStatus (kCall* VInit)(GoTool tool, kType type, kObject sensor, kAlloc allocator); 
    kStatus (kCall* VWrite)(GoTool tool, kXml xml, kXmlItem item); 
    kStatus (kCall* VRead)(GoTool tool, kXml xml, kXmlItem item);
} GoToolVTable; 

typedef struct GoToolClass
{
    kObjectClass base; 
    kObject sensor; 

    kSize index;   //represents position index in the Tools element
    kXml xml;
    kXmlItem xmlItem;

    kText128 name;
    k32s id;
    GoToolFormat format;
    
    kArrayList featureOutputs;
    kArrayList featureOptions;
    kArrayList featureNodesToMerge;

    kArrayList measurements;
    kArrayList measurementOptions;
    kArrayList measurementNodesToMerge;
    
    GoToolType typeId;
} GoToolClass; 

kDeclareVirtualClassEx(Go, GoTool, kObject)

#define GoTool_Class_(MODE)                     (kCastClass_(GoTool, MODE))

GoFx(kStatus) GoTool_Construct(GoTool* tool, kType type, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoTool_VInit(GoTool tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoTool_VRelease(GoTool tool);
GoFx(kStatus) GoTool_VRead(GoTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoTool_VWrite(GoTool tool, kXml xml, kXmlItem item); 
GoFx(kStatus) GoTool_Init(GoTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoTool_Read(GoTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoTool_Write(GoTool tool, kXml xml, kXmlItem item); 
GoFx(kStatus) GoTool_ReadMeasurements(GoTool tool, kXml xml, kXmlItem measurements, kText64 toolType);
GoFx(kStatus) GoTool_ReadFeatures(GoTool tool,kXml xml, const kChar* toolName, kXmlItem features);

GoFx(kObject) GoTool_Sensor(GoTool tool);
GoFx(kStatus) GoTool_SetIndex(GoTool tool, kSize index);

/**
* Parses stream options for the given tool.
*
* @public               @memberof GoTool
 * @version             Introduced in firmware 4.8.2.76
* @param    tool        GoTool object.
* @param    xml         kXml object.
* @param    item        kXmlItem object.
* @param    list        kArrayList object.
* @return   Operation status.
*/
GoFx(kStatus) GoToolUtil_ParseStreamOptions(GoTool tool, kXml xml, kXmlItem item, kArrayList list);

#endif

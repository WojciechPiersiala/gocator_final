/** 
 * @file    kImage.cpp
 *
 * @internal
 * Copyright (C) 2003-2020 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kImage.h>
#include <kApi/Data/kArrayProvider.h>
#include <kApi/Io/kFile.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Io/kDat5Serializer.h>

kBeginClassEx(k, kImage) 
    
    //serialization versions
    kAddPrivateVersion(kImage, "kdat5", "5.0.0.0", "31-3", WriteDat5V3, ReadDat5V3)
    kAddPrivateVersion(kImage, "kdat6", "5.7.1.0", "kImage-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddPrivateVMethod(kImage, kObject, VRelease)
    kAddPrivateVMethod(kImage, kObject, VInitCloneEx)
    kAddPrivateVMethod(kImage, kObject, VSize)
    kAddPrivateVMethod(kImage, kObject, VAllocTraits)

    //array provider interface 
    kAddInterface(kImage, kArrayProvider)
    kAddPrivateIVMethod(kImage, kArrayProvider, VDefaultConstruct, DefaultConstruct)
    kAddPrivateIVMethod(kImage, kArrayProvider, VImitate, Imitate)
    kAddPrivateIVMethod(kImage, kArrayProvider, VAssign, Assign)
    kAddIVMethod(kImage, kArrayProvider, VItemType, PixelType)
    kAddIVMethod(kImage, kArrayProvider, VCount, Area)
    kAddIVMethod(kImage, kArrayProvider, VData, Data)
    kAddIVMethod(kImage, kArrayProvider, VDataSize, DataSize)
    kAddIVMethod(kImage, kArrayProvider, VDataAlloc, DataAlloc)

kEndClassEx() 

kFx(kStatus) kImage_ConstructEx(kImage* image, kType pixelType, kSize width, kSize height, kAlloc allocator, kAlloc valueAllocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kAlloc valueAlloc = kAlloc_Fallback(valueAllocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kImage), image)); 

    if (!kSuccess(status = xkImage_Init(*image, kTypeOf(kImage), pixelType, width, height, alloc, valueAlloc)))
    {
        kAlloc_FreeRef(alloc, image); 
    }

    return status; 
} 

kFx(kStatus) kImage_Construct(kImage* image, kType pixelType, kSize width, kSize height, kAlloc allocator)
{
    return kImage_ConstructEx(image, pixelType, width, height, allocator, allocator);
}

kFx(kStatus) xkImage_DefaultConstruct(kImage* image, kAlloc objectAlloc, kAlloc valueAlloc)
{
    return kImage_ConstructEx(image, kNULL, 0, 0, objectAlloc, valueAlloc);
}

kFx(kStatus) kImage_Import(kImage* image, const kChar* fileName, kAlloc allocator)
{
    return xkBmpLoad(image, fileName, allocator); 
}

kFx(kStatus) kImage_Export(kImage image, const kChar* fileName)
{
    return xkBmpSave(image, fileName); 
}

kFx(kStatus) xkImage_Init(kImage image, kType classType, kType pixelType, kSize width, kSize height, kAlloc alloc, kAlloc valueAlloc)
{
    kObjR(kImage, image);
    kStatus status = kOK; 

    kCheckArgs(kIsNull(pixelType) || kType_IsValue(pixelType)); 

    kCheck(kObject_Init(image, classType, alloc)); 
    
    obj->dataAlloc = valueAlloc;
    obj->pixelType = kNULL;
    obj->pixelSize = 0;
    obj->allocSize = 0;
    obj->pixels = kNULL;
    obj->stride = 0;
    obj->width = 0;
    obj->height = 0;
    obj->isAttached = kFALSE;
    obj->format = 0;
    obj->cfa = 0;

    kTry
    {
        kTest(kImage_Allocate(image, pixelType, width, height));  
    }
    kCatch(&status)
    {
        xkImage_VRelease(image); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkImage_VInitCloneEx(kImage image, kImage source, kAlloc objectAlloc, kAlloc valueAlloc, kObject context)
{
    kObjR(kImage, image);
    kObjN(kImage, sourceObj, source); 
    kStatus status = kOK; 

    kCheck(kObject_Init(image, kObject_Type(source), objectAlloc));     

    obj->dataAlloc = valueAlloc;
    obj->pixelType = kNULL;
    obj->pixelSize = 0;
    obj->allocSize = 0;
    obj->pixels = kNULL;
    obj->stride = 0;
    obj->width = 0;
    obj->height = 0;
    obj->isAttached = kFALSE;
    obj->format = sourceObj->format;
    obj->cfa = sourceObj->cfa;
 
    kTry
    {
        kTest(xkImage_Imitate(image, source)); 

        kTest(xkImage_CopyPixels(image, source, context)); 
    }
    kCatch(&status)
    {
        xkImage_VRelease(image); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkImage_VRelease(kImage image)
{
    kObj(kImage, image); 

    if (!obj->isAttached)
    {
        kCheck(kAlloc_Free(obj->dataAlloc, obj->pixels)); 
    }

    kCheck(kObject_VRelease(image)); 

    return kOK; 
}

kFx(kAllocTrait) xkImage_VAllocTraits(kImage image)
{
    kObj(kImage, image);

    return kAlloc_Traits(kObject_Alloc(image)) | kAlloc_Traits(kImage_DataAlloc(image));
}

kFx(kStatus) xkImage_WriteDat5V3(kImage image, kSerializer serializer)
{
    kObj(kImage, image); 
    kTypeVersion itemVersion; 
    kSize i; 

    kCheckState(!kAlloc_IsForeign(obj->dataAlloc));
    
    kCheck(kSerializer_WriteSize(serializer, obj->width)); 
    kCheck(kSerializer_WriteSize(serializer, obj->height)); 
    kCheck(kSerializer_Write32s(serializer, obj->cfa)); 

    //works around a bug/limitation in serialization version 5.3 -- pixel type version not written
    kCheck(kObject_Is(serializer, kTypeOf(kDat5Serializer))); 
    kCheck(xkDat5Serializer_WriteTypeWithoutVersion(serializer, obj->pixelType, &itemVersion));

    for (i = 0; i < obj->height; ++i)
    {
        void* row = kImage_RowAt(image, i); 

        kCheck(kSerializer_WriteSize(serializer, obj->width)); 
        kCheck(kSerializer_WriteType(serializer, obj->pixelType, &itemVersion));
        kCheck(kSerializer_WriteItems(serializer, obj->pixelType, itemVersion, row, obj->width)); 
    }

    return kOK; 
}

kFx(kStatus) xkImage_ReadDat5V3(kImage image, kSerializer serializer, kAlloc allocator)
{
    kObjR(kImage, image); 
    kSize width = 0, height = 0; 
    k32s cfa; 
    kTypeVersion itemVersion;
    kType pixelType = kNULL;            
    kStatus status; 
    kSize i; 

    kCheck(kSerializer_ReadSize(serializer, &width)); 
    kCheck(kSerializer_ReadSize(serializer, &height)); 
    kCheck(kSerializer_Read32s(serializer, &cfa)); 

    //works around a bug/limitation in serialization version 5.3 -- pixel type version not written
    kCheck(kObject_Is(serializer, kTypeOf(kDat5Serializer))); 
    kCheck(xkDat5Serializer_ReadTypeExplicitVersion(serializer, 0, &pixelType, &itemVersion)); 

    kCheck(xkImage_Init(image, kTypeOf(kImage), pixelType, width, height, allocator, allocator)); 

    kTry
    {         
        obj->cfa = cfa; 

        for (i = 0; i < obj->height; ++i)
        {
            void* row = kImage_RowAt(image, i); 

            kTest(kSerializer_ReadSize(serializer, &width)); 
            kTest(kSerializer_ReadType(serializer, &pixelType, &itemVersion));
            kTest(kSerializer_ReadItems(serializer, pixelType, itemVersion, row, obj->width)); 
        }
    }
    kCatch(&status)
    {
        xkImage_VRelease(image); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkImage_WriteDat6V0(kImage image, kSerializer serializer)
{
    kObj(kImage, image); 
    kTypeVersion itemVersion; 
    kSize i; 

    kCheckState(!kAlloc_IsForeign(obj->dataAlloc));

    kCheck(kSerializer_WriteType(serializer, obj->pixelType, &itemVersion));
    kCheck(kSerializer_WriteSize(serializer, obj->width)); 
    kCheck(kSerializer_WriteSize(serializer, obj->height));
    kCheck(kSerializer_Write32s(serializer, obj->cfa)); 
    kCheck(kSerializer_Write32s(serializer, obj->format)); 

    //reserve space for optional fields
    kCheck(kSerializer_Write16u(serializer, 0)); 

    if ((obj->width*obj->pixelSize) == obj->stride)
    {
        kCheck(kSerializer_WriteItems(serializer, obj->pixelType, itemVersion, obj->pixels, obj->width*obj->height));       
    }
    else
    {
        for (i = 0; i < obj->height; ++i)
        {
            void* row = kImage_RowAt(image, i); 

            kCheck(kSerializer_WriteItems(serializer, obj->pixelType, itemVersion, row, obj->width)); 
        }
    }

    return kOK; 
}

kFx(kStatus) xkImage_ReadDat6V0(kImage image, kSerializer serializer, kAlloc allocator)
{
    kObjR(kImage, image); 
    kType pixelType = kNULL;            
    kTypeVersion itemVersion; 
    kSize width = 0, height = 0; 
    k16u optional = 0; 
    kStatus status; 
    kSize i; 

    kCheck(kSerializer_ReadType(serializer, &pixelType, &itemVersion));
    kCheck(kSerializer_ReadSize(serializer, &width)); 
    kCheck(kSerializer_ReadSize(serializer, &height)); 

    kCheck(xkImage_Init(image, kTypeOf(kImage), pixelType, width, height, allocator, allocator)); 

    kTry
    {
        kTest(kSerializer_Read32s(serializer, &obj->cfa)); 
        kTest(kSerializer_Read32s(serializer, &obj->format)); 

        //consume optional fields
        kTest(kSerializer_Read16u(serializer, &optional)); 
        kTest(kSerializer_AdvanceRead(serializer, optional)); 

        if ((obj->width*obj->pixelSize) == obj->stride)
        {
            kTest(kSerializer_ReadItems(serializer, pixelType, itemVersion, obj->pixels, obj->width*obj->height));
        }
        else
        {
            for (i = 0; i < obj->height; ++i)
            {
                void* row = kImage_RowAt(image, i);

                kTest(kSerializer_ReadItems(serializer, pixelType, itemVersion, row, obj->width));
            }
        }
    }
    kCatch(&status)
    {
        xkImage_VRelease(image); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkImage_CopyPixels(kImage image, kImage source, kObject context)
{
    kObj(kImage, image); 
    kObjN(kImage, sourceObj, source); 
    kSize i; 

    kCheckArgs((obj->width == sourceObj->width) && (obj->height == sourceObj->height) && (obj->pixelSize == sourceObj->pixelSize)); 

    if (obj->stride == sourceObj->stride)
    {
        kCheck(kAlloc_Copy(obj->dataAlloc, obj->pixels, sourceObj->dataAlloc, sourceObj->pixels, obj->stride*obj->height, context));
    }
    else
    {
        for (i = 0; i < obj->height; ++i)
        {
            void* row = kImage_RowAt(image, i);
            void* srcRow = kImage_RowAt(source, i);

            kCheck(kAlloc_Copy(obj->dataAlloc, row, sourceObj->dataAlloc, srcRow, obj->pixelSize*obj->width, context));
        }
    }

    return kOK; 
}

kFx(kStatus) kImage_Allocate(kImage image, kType pixelType, kSize width, kSize height)
{
    kObj(kImage, image); 
    kSize newWidth = width; 
    kSize newHeight = height; 
    kType newPixelType = kIsNull(pixelType) ? kTypeOf(kVoid) : pixelType; 
    kSize newPixelSize = kType_Size(newPixelType); 
    kSize newStride = kSize_Align(newWidth * newPixelSize, xkIMAGE_ALIGNMENT); 
    kSize newSize = kMax_(obj->allocSize, newHeight*newStride); 

    if (obj->isAttached)
    {
        obj->pixels = kNULL; 
        obj->isAttached = kFALSE; 
    }

    if (newSize > obj->allocSize)
    {
        void* oldPixels = obj->pixels; 
        void* newPixels = kNULL; 

        kCheck(kAlloc_Get(obj->dataAlloc, newSize, &newPixels));
        
        obj->pixels = newPixels; 
        obj->allocSize = newSize; 

        kCheck(kAlloc_Free(obj->dataAlloc, oldPixels));
    }
        
    obj->pixelType = newPixelType; 
    obj->pixelSize = newPixelSize; 
    obj->width = newWidth; 
    obj->height = newHeight; 
    obj->stride = newStride; 
    
    return kOK; 
}

kFx(kStatus) kImage_Attach(kImage image, void* pixels, kType pixelType, kSize width, kSize height, kSize stride)
{
    kObj(kImage, image); 

    if (!obj->isAttached)
    {
        kCheck(kAlloc_FreeRef(obj->dataAlloc, &obj->pixels));
    }
    
    obj->pixelType = kIsNull(pixelType) ? kTypeOf(kVoid) : pixelType; 
    obj->pixelSize = kType_Size(obj->pixelType); 
    obj->allocSize = 0; 
    obj->pixels = pixels; 
    obj->stride = stride; 
    obj->width = width; 
    obj->height = height; 
    obj->isAttached = kTRUE; 

    return kOK; 
}

kFx(kStatus) xkImage_Imitate(kImage image, kArray1 source)
{
    kObj(kImage, image); 

    kCheck(kImage_Allocate(image, kImage_PixelType(source), kImage_Width(source), kImage_Height(source)));

    obj->format = kImage_PixelFormat(source); 
    obj->cfa = kImage_Cfa(source); 

    return kOK;
}

kFx(kStatus) xkImage_Assign(kImage image, kImage source, kObject context)
{
    kObj(kImage, image); 

    kCheck(xkImage_Imitate(image, source));

    kCheck(xkImage_CopyPixels(image, source, context)); 

    return kOK;   
}

kFx(kStatus) kImage_Zero(kImage image)
{
    kObj(kImage, image); 

    kCheckArgs(!kAlloc_IsForeign(obj->dataAlloc));

    return kMemSet(obj->pixels, 0, kImage_DataSize(image)); 
}

kFx(kStatus) kImage_SetPixel(kImage image, kSize x, kSize y, const void* pixel)
{
    kObj(kImage, image); 

    kAssert(!kAlloc_IsForeign(obj->dataAlloc));
    kCheckArgs((x < obj->width) && (y < obj->height)); 

    kValue_Import(obj->pixelType, kImage_DataAt(image, (kSSize)x, (kSSize)y), pixel); 

    return kOK; 
}

kFx(kStatus) kImage_Pixel(kImage image, kSize x, kSize y, void* pixel)
{
    kObj(kImage, image); 

    kAssert(!kAlloc_IsForeign(obj->dataAlloc));
    kCheckArgs((x < obj->width) && (y < obj->height)); 

    kItemCopy(pixel, kImage_DataAt(image, (kSSize)x, (kSSize)y), obj->pixelSize); 

    return kOK; 
}


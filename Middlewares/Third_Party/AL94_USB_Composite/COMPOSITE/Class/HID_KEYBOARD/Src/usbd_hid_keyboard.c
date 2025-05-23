/**
  ******************************************************************************
  * @file    usbd_hid.c
  * @author  MCD Application Team
  * @brief   This file provides the HID core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                HID Class  Description
  *          ===================================================================
  *           This module manages the HID class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - The Keyboard protocol
  *             - Usage Page : Generic Desktop
  *             - Collection : Application
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid_keyboard.h"
#include "usbd_ctlreq.h"

#define _HID_KEYBOARD_IN_EP 0x81U
#define _HID_KEYBOARD_ITF_NBR 0x00
#define _HID_KEYBOARD_STR_DESC_IDX 0x00U

uint8_t HID_KEYBOARD_IN_EP = _HID_KEYBOARD_IN_EP;
uint8_t HID_KEYBOARD_ITF_NBR = _HID_KEYBOARD_ITF_NBR;
uint8_t HID_KEYBOARD_STR_DESC_IDX = _HID_KEYBOARD_STR_DESC_IDX;

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_HID
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_HID_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_HID_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_HID_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_HID_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);

/**
  * @}
  */

/** @defgroup USBD_HID_Private_Variables
  * @{
  */

static USBD_HID_Keyboard_HandleTypeDef USBD_HID_KBD_Instace;

USBD_ClassTypeDef USBD_HID_KEYBOARD =
    {
        USBD_HID_Init,
        USBD_HID_DeInit,
        USBD_HID_Setup,
        NULL,            /* EP0_TxSent */
        NULL,            /* EP0_RxReady */
        USBD_HID_DataIn, /* DataIn */
        NULL,            /* DataOut */
        NULL,            /* SOF */
        NULL,
        NULL,
        USBD_HID_GetHSCfgDesc,
        USBD_HID_GetFSCfgDesc,
        USBD_HID_GetOtherSpeedCfgDesc,
        USBD_HID_GetDeviceQualifierDesc,
};

/* USB HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_KEYBOARD_CfgFSDesc[HID_KEYBOARD_CONFIG_DESC_SIZE] __ALIGN_END =
    {
        0x09,                          /* bLength: Configuration Descriptor size */
        USB_DESC_TYPE_CONFIGURATION,   /* bDescriptorType: Configuration */
        HID_KEYBOARD_CONFIG_DESC_SIZE, /* wTotalLength: Bytes returned */
        0x00,
        0x01, /* bNumInterfaces: 1 interface */
        0x01, /* bConfigurationValue: Configuration value */
        0x00, /* iConfiguration: Index of string descriptor describing the configuration */
#if (USBD_SELF_POWERED == 1U)
        0xE0, /* bmAttributes: Bus Powered according to user configuration */
#else
        0xA0, /* bmAttributes: Bus Powered according to user configuration */
#endif
        USBD_MAX_POWER, /* MaxPower 100 mA: this current is used for detecting Vbus */

        /************** Descriptor of Keyboard interface ****************/
        /* 09 */
        0x09,                       /* bLength: Interface Descriptor size */
        USB_DESC_TYPE_INTERFACE,    /* bDescriptorType: Interface descriptor type */
        _HID_KEYBOARD_ITF_NBR,      /* bInterfaceNumber: Number of Interface */
        0x00,                       /* bAlternateSetting: Alternate setting */
        0x01,                       /* bNumEndpoints */
        0x03,                       /* bInterfaceClass: HID */
        0x01,                       /* bInterfaceSubClass : 1=BOOT, 0=no boot */
        0x01,                       /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
        _HID_KEYBOARD_STR_DESC_IDX, /* iInterface: Index of string descriptor */
        /******************** Descriptor of Keyboard HID ********************/
        /* 18 */
        0x09,                         /* bLength: HID Descriptor size */
        HID_KEYBOARD_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
        0x11,                         /* bcdHID: HID Class Spec release number */
        0x01,
        0x00,                          /* bCountryCode: Hardware target country */
        0x01,                          /* bNumDescriptors: Number of HID class descriptors to follow */
        0x22,                          /* bDescriptorType */
        HID_KEYBOARD_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
        0x00,
        /******************** Descriptor of Keyboard endpoint ********************/
        /* 27 */
        0x07,                   /* bLength: Endpoint Descriptor size */
        USB_DESC_TYPE_ENDPOINT, /* bDescriptorType:*/
        _HID_KEYBOARD_IN_EP,    /* bEndpointAddress: Endpoint Address (IN) */
        0x03,                   /* bmAttributes: Interrupt endpoint */
        HID_KEYBOARD_EPIN_SIZE, /* wMaxPacketSize: 4 Byte max */
        0x00,
        HID_KEYBOARD_FS_BINTERVAL, /* bInterval: Polling Interval */
                                   /* 34 */
};

/* USB HID device HS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_KEYBOARD_CfgHSDesc[HID_KEYBOARD_CONFIG_DESC_SIZE] __ALIGN_END =
    {
        0x09,                          /* bLength: Configuration Descriptor size */
        USB_DESC_TYPE_CONFIGURATION,   /* bDescriptorType: Configuration */
        HID_KEYBOARD_CONFIG_DESC_SIZE, /* wTotalLength: Bytes returned */
        0x00,
        0x01, /* bNumInterfaces: 1 interface */
        0x01, /* bConfigurationValue: Configuration value */
        0x00, /* iConfiguration: Index of string descriptor describing the configuration */
#if (USBD_SELF_POWERED == 1U)
        0xE0, /* bmAttributes: Bus Powered according to user configuration */
#else
        0xA0, /* bmAttributes: Bus Powered according to user configuration */
#endif
        USBD_MAX_POWER, /* MaxPower 100 mA: this current is used for detecting Vbus */

        /************** Descriptor of Keyboard interface ****************/
        /* 09 */
        0x09,                       /* bLength: Interface Descriptor size */
        USB_DESC_TYPE_INTERFACE,    /* bDescriptorType: Interface descriptor type */
        _HID_KEYBOARD_ITF_NBR,      /* bInterfaceNumber: Number of Interface */
        0x00,                       /* bAlternateSetting: Alternate setting */
        0x01,                       /* bNumEndpoints */
        0x03,                       /* bInterfaceClass: HID */
        0x01,                       /* bInterfaceSubClass : 1=BOOT, 0=no boot */
        0x01,                       /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
        _HID_KEYBOARD_STR_DESC_IDX, /* iInterface: Index of string descriptor */
        /******************** Descriptor of Keyboard HID ********************/
        /* 18 */
        0x09,                         /* bLength: HID Descriptor size */
        HID_KEYBOARD_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
        0x11,                         /* bcdHID: HID Class Spec release number */
        0x01,
        0x00,                          /* bCountryCode: Hardware target country */
        0x01,                          /* bNumDescriptors: Number of HID class descriptors to follow */
        0x22,                          /* bDescriptorType */
        HID_KEYBOARD_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
        0x00,
        /******************** Descriptor of Keyboard endpoint ********************/
        /* 27 */
        0x07,                   /* bLength: Endpoint Descriptor size */
        USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
        _HID_KEYBOARD_IN_EP,    /* bEndpointAddress: Endpoint Address (IN) */
        0x03,                   /* bmAttributes: Interrupt endpoint */
        HID_KEYBOARD_EPIN_SIZE, /* wMaxPacketSize: 4 Byte max */
        0x00,
        HID_KEYBOARD_HS_BINTERVAL, /* bInterval: Polling Interval */
                                   /* 34 */
};

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_KEYBOARD_Desc[HID_KEYBOARD_DESC_SIZE] __ALIGN_END =
    {
        /* 18 */
        0x09,                         /* bLength: HID Descriptor size */
        HID_KEYBOARD_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
        0x11,                         /* bcdHID: HID Class Spec release number */
        0x01,
        0x00,                          /* bCountryCode: Hardware target country */
        0x01,                          /* bNumDescriptors: Number of HID class descriptors to follow */
        0x22,                          /* bDescriptorType */
        HID_KEYBOARD_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
        0x00,
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
    {
        USB_LEN_DEV_QUALIFIER_DESC,
        USB_DESC_TYPE_DEVICE_QUALIFIER,
        0x00,
        0x02,
        0x00,
        0x00,
        0x00,
        0x40,
        0x01,
        0x00,
};

/*  HID keyboard report descriptor */
__ALIGN_BEGIN static uint8_t HID_KEYBOARD_ReportDesc[HID_KEYBOARD_REPORT_DESC_SIZE] __ALIGN_END =
    {
        0x05, 0x01,
        0x09, 0x06,
        0xA1, 0x01,
        0x85, 0x01,
        0x05, 0x07,
        0x19, 0xE0,
        0x29, 0xE7,
        0x15, 0x00,
        0x25, 0x01,
        0x75, 0x01,
        0x95, 0x08,
        0x81, 0x02,
        0x75, 0x08,
        0x95, 0x01,
        0x81, 0x01,
        0x05, 0x07,
        0x19, 0x00,
        0x29, 0x65,
        0x15, 0x00,
        0x25, 0x65,
        0x75, 0x08,
        0x95, 0x05,
        0x81, 0x00,
        0xC0, 0x05,
        0x0C, 0x09,
        0x01, 0xA1,
        0x01, 0x85,
        0x02, 0x19,
        0x00, 0x2A,
        0x3C, 0x02,
        0x15, 0x00,
        0x26, 0x3C,
        0x02, 0x95,
        0x01, 0x75,
        0x10, 0x81,
        0x00, 0xC0,
        0x05, 0x01,
        0x09, 0x80,
        0xA1, 0x01,
        0x85, 0x03,
        0x19, 0x81,
        0x29, 0x83,
        0x15, 0x00,
        0x25, 0x01,
        0x75, 0x01,
        0x95, 0x03,
        0x81, 0x02,
        0x95, 0x05,
        0x81, 0x01,
        0xC0, 0x06,
        0x01, 0xFF,
        0x09, 0x01,
        0xA1, 0x01,
        0x85, 0x04,
        0x95, 0x01,
        0x75, 0x08,
        0x15, 0x01,
        0x25, 0x0A,
        0x09, 0x20,
        0xB1, 0x03,
        0x09, 0x23,
        0xB1, 0x03,
        0x25, 0x4F,
        0x09, 0x21,
        0xB1, 0x03,
        0x25, 0x30,
        0x09, 0x22,
        0xB1, 0x03,
        0x95, 0x03,
        0x09, 0x24,
        0xB1, 0x03,
        0xC0, 0x06,
        0x01, 0xFF,
        0x09, 0x01,
        0xA1, 0x01,
        0x85, 0x05,
        0x95, 0x01,
        0x75, 0x08,
        0x15, 0x01,
        0x25, 0x0A,
        0x09, 0x20,
        0xB1, 0x03,
        0x09, 0x23,
        0xB1, 0x03,
        0x25, 0x4F,
        0x09, 0x21,
        0xB1, 0x03,
        0x25, 0x30,
        0x09, 0x22,
        0xB1, 0x03,
        0x95, 0x03,
        0x09, 0x24,
        0xB1, 0x03,
        0xC0};

/**
  * @}
  */

/** @defgroup USBD_HID_Private_Functions
  * @{
  */

/**
  * @brief  USBD_HID_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_HID_Keyboard_HandleTypeDef *hhid;

  hhid = &USBD_HID_KBD_Instace;

  if (hhid == NULL)
  {
    pdev->pClassData_HID_Keyboard = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassData_HID_Keyboard = (void *)hhid;

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    pdev->ep_in[HID_KEYBOARD_IN_EP & 0xFU].bInterval = HID_KEYBOARD_HS_BINTERVAL;
  }
  else /* LOW and FULL-speed endpoints */
  {
    pdev->ep_in[HID_KEYBOARD_IN_EP & 0xFU].bInterval = HID_KEYBOARD_FS_BINTERVAL;
  }

  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, HID_KEYBOARD_IN_EP, USBD_EP_TYPE_INTR, HID_KEYBOARD_EPIN_SIZE);
  pdev->ep_in[HID_KEYBOARD_IN_EP & 0xFU].is_used = 1U;

  hhid->state = KEYBOARD_HID_IDLE;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_DeInit
  *         DeInitialize the HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  /* Close HID EPs */
  (void)USBD_LL_CloseEP(pdev, HID_KEYBOARD_IN_EP);
  pdev->ep_in[HID_KEYBOARD_IN_EP & 0xFU].is_used = 0U;
  pdev->ep_in[HID_KEYBOARD_IN_EP & 0xFU].bInterval = 0U;

  /* Free allocated memory */
  if (pdev->pClassData_HID_Keyboard != NULL)
  {
#if (0)
    (void)USBD_free(pdev->pClassData_HID_Keyboard);
#endif
    pdev->pClassData_HID_Keyboard = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_HID_Keyboard_HandleTypeDef *hhid = (USBD_HID_Keyboard_HandleTypeDef *)pdev->pClassData_HID_Keyboard;
  USBD_StatusTypeDef ret = USBD_OK;
  uint16_t len;
  uint8_t *pbuf;
  uint16_t status_info = 0U;

  if (hhid == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS:
    switch (req->bRequest)
    {
    case HID_KEYBOARD_REQ_SET_PROTOCOL:
      hhid->Protocol = (uint8_t)(req->wValue);
      break;

    case HID_KEYBOARD_REQ_GET_PROTOCOL:
      (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->Protocol, 1U);
      break;

    case HID_KEYBOARD_REQ_SET_IDLE:
      hhid->IdleState = (uint8_t)(req->wValue >> 8);
      break;

    case HID_KEYBOARD_REQ_GET_IDLE:
      (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->IdleState, 1U);
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_STATUS:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
      }
      else
      {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_GET_DESCRIPTOR:
      if ((req->wValue >> 8) == HID_KEYBOARD_REPORT_DESC)
      {
        len = MIN(HID_KEYBOARD_REPORT_DESC_SIZE, req->wLength);
        pbuf = HID_KEYBOARD_ReportDesc;
      }
      else if ((req->wValue >> 8) == HID_KEYBOARD_DESCRIPTOR_TYPE)
      {
        pbuf = USBD_HID_KEYBOARD_Desc;
        len = MIN(HID_KEYBOARD_DESC_SIZE, req->wLength);
      }
      else
      {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
        break;
      }
      (void)USBD_CtlSendData(pdev, pbuf, len);
      break;

    case USB_REQ_GET_INTERFACE:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->AltSetting, 1U);
      }
      else
      {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_SET_INTERFACE:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        hhid->AltSetting = (uint8_t)(req->wValue);
      }
      else
      {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_CLEAR_FEATURE:
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;

  default:
    USBD_CtlError(pdev, req);
    ret = USBD_FAIL;
    break;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_HID_GetCfgFSDesc
  *         return FS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_KEYBOARD_CfgFSDesc);

  return USBD_HID_KEYBOARD_CfgFSDesc;
}

/**
  * @brief  USBD_HID_GetCfgHSDesc
  *         return HS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_KEYBOARD_CfgHSDesc);

  return USBD_HID_KEYBOARD_CfgHSDesc;
}

/**
  * @brief  USBD_HID_GetOtherSpeedCfgDesc
  *         return other speed configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_KEYBOARD_CfgFSDesc);

  return USBD_HID_KEYBOARD_CfgFSDesc;
}

/**
  * @brief  USBD_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(epnum);
  /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_HID_Keyboard_HandleTypeDef *)pdev->pClassData_HID_Keyboard)->state = KEYBOARD_HID_IDLE;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_DeviceQualifierDesc);

  return USBD_HID_DeviceQualifierDesc;
}

/**
  * @brief  USBD_HID_SendReport
  *         Send HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t USBD_HID_Keyboard_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len)
{
  USBD_HID_Keyboard_HandleTypeDef *hhid = (USBD_HID_Keyboard_HandleTypeDef *)pdev->pClassData_HID_Keyboard;

  if (hhid == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (hhid->state == KEYBOARD_HID_IDLE)
    {
      hhid->state = KEYBOARD_HID_BUSY;
      (void)USBD_LL_Transmit(pdev, HID_KEYBOARD_IN_EP, report, len);
    }
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_GetPollingInterval
  *         return polling interval from endpoint descriptor
  * @param  pdev: device instance
  * @retval polling interval
  */
uint32_t USBD_HID_Keyboard_GetPollingInterval(USBD_HandleTypeDef *pdev)
{
  uint32_t polling_interval;

  /* HIGH-speed endpoints */
  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Sets the data transfer polling interval for high speed transfers.
     Values between 1..16 are allowed. Values correspond to interval
     of 2 ^ (bInterval-1). This option (8 ms, corresponds to HID_HS_BINTERVAL */
    polling_interval = (((1U << (HID_KEYBOARD_HS_BINTERVAL - 1U))) / 8U);
  }
  else /* LOW and FULL-speed endpoints */
  {
    /* Sets the data transfer polling interval for low and full
    speed transfers */
    polling_interval = HID_KEYBOARD_FS_BINTERVAL;
  }

  return ((uint32_t)(polling_interval));
}

void USBD_Update_HID_KBD_DESC(uint8_t *desc, uint8_t itf_no, uint8_t in_ep, uint8_t str_idx)
{
  desc[11] = itf_no;
  desc[17] = str_idx;
  desc[29] = in_ep;

  HID_KEYBOARD_IN_EP = in_ep;
  HID_KEYBOARD_ITF_NBR = itf_no;
  HID_KEYBOARD_STR_DESC_IDX = str_idx;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

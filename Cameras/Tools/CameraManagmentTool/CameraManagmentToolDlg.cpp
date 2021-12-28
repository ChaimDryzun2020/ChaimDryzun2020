
// CameraManagmentToolDlg.cpp : implementation file
//

#include "pch.h"
#include "framework.h"
#include "CameraManagmentTool.h"
#include "CameraManagmentToolDlg.h"
#include "afxdialogex.h"

//#include <pcl/visualization/cloud_viewer.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CCameraManagmentToolDlg dialog

CCameraManagmentToolDlg::CCameraManagmentToolDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_CAMERAMANAGMENTTOOL_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CCameraManagmentToolDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_CAMERAS_LIST, m_CComboBox);
}

BEGIN_MESSAGE_MAP(CCameraManagmentToolDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_MOUSEMOVE()
	ON_WM_QUERYDRAGICON()	
	ON_CBN_SELCHANGE(IDC_CAMERAS_LIST, &CCameraManagmentToolDlg::OnCbnSelchangeCombo1)
	ON_BN_CLICKED(ID_RGB_IMAGE, &CCameraManagmentToolDlg::OnBnClickedButtonRGBimage)
	ON_BN_CLICKED(ID_DEPTH_MAP, &CCameraManagmentToolDlg::OnBnClickedButtonDepthMap)
	ON_BN_CLICKED(ID_HEAT_MAP, &CCameraManagmentToolDlg::OnBnClickedButtonHeatMap)
END_MESSAGE_MAP()


// CCameraManagmentToolDlg message handlers

BOOL CCameraManagmentToolDlg::PreTranslateMessage(MSG* pMsg)
{
	if (pMsg->message == WM_KEYDOWN)
	{
		if (pMsg->wParam == VK_RETURN)
		{
			pMsg->wParam = NULL;
		}
	}

	return CDialog::PreTranslateMessage(pMsg);
}

BOOL CCameraManagmentToolDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	SetWindowText(L"SkillReal Camera Management Tool - version 1.0");

	m_iModelNum = 0;
	UpdateCamerasList();

	cv::namedWindow("Image");

	return TRUE;  // return TRUE  unless you set the focus to a control
}


// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CCameraManagmentToolDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CCameraManagmentToolDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CCameraManagmentToolDlg::UpdateCamerasList()
{
	m_CMcamerasMang.CamerasScan();
	
	int CamerasNum = m_CMcamerasMang.GetCamerasNum();

	if ((m_iModelNum < 0) || (m_iModelNum >= CamerasNum))
		m_iModelNum = 0;

	m_CComboBox.Clear();
	m_vCamerasNames.clear();

	CameraInterfacePtr Camera_i;
	std::string CameraFullName;
	for (int Index = 0; Index < CamerasNum; Index++)
	{
		CameraFullName = "";
		Camera_i = m_CMcamerasMang.GetCamera(Index);
		CameraFullName = Camera_i->GetName();
		CameraFullName.append("_");
		CameraFullName.append(Camera_i->GetID());
		m_vCamerasNames.push_back(CameraFullName);
		LPWSTR  ws = new wchar_t[CameraFullName.size() + 1];
		copy(CameraFullName.begin(), CameraFullName.end(), ws);
		ws[CameraFullName.size()] = 0; // zero at the end

		m_CComboBox.AddString(ws);
	}
	m_CComboBox.SetCurSel(m_iModelNum);
}

void CCameraManagmentToolDlg::OnCbnSelchangeCombo1()
{
	m_iModelNum = m_CComboBox.GetCurSel();
}

void CCameraManagmentToolDlg::OnBnClickedButtonRGBimage()
{
	CameraInterfacePtr Camera_i;
	Camera_i = m_CMcamerasMang.GetCamera(m_iModelNum);
	if (Camera_i->SupportsRGB())
	{
		RGBATexture ImageRGB;
		ImageRGB = Camera_i->UpdateRGBAImage();
		cv::imshow("Image", ImageRGB.m_mRGBA);
	}
}

void CCameraManagmentToolDlg::OnBnClickedButtonDepthMap()
{
	CameraInterfacePtr Camera_i;
	Camera_i = m_CMcamerasMang.GetCamera(m_iModelNum);
	DepthTexture DepthImage;
	DepthImage = Camera_i->GetDepthMap();
	cv::imshow("Image", DepthImage.m_mDepth);
}

void CCameraManagmentToolDlg::OnBnClickedButtonHeatMap()
{
	CameraInterfacePtr Camera_i;
	Camera_i = m_CMcamerasMang.GetCamera(m_iModelNum);
	RGBATexture HeatMap;
	HeatMap = Camera_i->GetHeatMap();
	cv::imshow("Image", HeatMap.m_mRGBA);
}

void CCameraManagmentToolDlg::OnBnClickedButtonPointCloud()
{
	CameraInterfacePtr Camera_i;
	Camera_i = m_CMcamerasMang.GetCamera(m_iModelNum);
	Camera_i->GetPointsCloud();
	
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	//viewer.showCloud(cloud);
	//while (!viewer.wasStopped())
	//{
	//}
}


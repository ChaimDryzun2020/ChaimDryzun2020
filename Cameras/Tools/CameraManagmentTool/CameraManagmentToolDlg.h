// CameraManagmentToolDlg.h : header file
//

#pragma once
#include <ctime>

#include <memory>
#include <vector>

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "CamerasManagementClass.h"

// CCameraManagmentToolDlg dialog
class CCameraManagmentToolDlg : public CDialogEx
{
// Construction
public:
	CCameraManagmentToolDlg(CWnd* pParent = nullptr);	// standard constructor
	~CCameraManagmentToolDlg() { cv::destroyAllWindows(); }
		
// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_CAMERAMANAGMENTTOOL_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	virtual BOOL PreTranslateMessage(MSG* pMsg);

	// Generated message map functions
	virtual BOOL OnInitDialog();	
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

private:
	CameraManagement m_CMcamerasMang;
	std::vector<std::string> m_vCamerasNames;
	int m_iModelNum;
	CComboBox m_CComboBox;

	void UpdateCamerasList();

public:
	
	afx_msg void OnCbnSelchangeCombo1();

	afx_msg void OnBnClickedButtonRGBimage();
	afx_msg void OnBnClickedButtonDepthMap();
	afx_msg void OnBnClickedButtonHeatMap();
	afx_msg void OnBnClickedButtonPointCloud();

};

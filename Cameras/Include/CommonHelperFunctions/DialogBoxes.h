// DialogBoxes.h
// This is the Header file for all kind of dialog boxes
//
// First created by Chaim Dryzun at 11.04.2021

#pragma once

#include "Constants.h"

namespace DialogBoxes
{
	// Opening a dialog box for loading a file
	std::string LoadFileDialogBox(const std::string& sExtension);

	// Opening a dialog box for saving a file
	std::string SaveFileDialogBox(const std::string& sExtension);

	// Opening an error message dialog box
	void DisplayErrorMessageBox(const std::string& sTitle);

	// Opening a general message dialog box
	int DisplayMessageBox(const std::string& sMessage, const std::string& sTitle);

	// Extracting only the Path from a full filename
	std::string GetFilesPath(std::string filePath);
}

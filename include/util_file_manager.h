# pragma once
#include "common.h"
#include "util_serializable.h"

/*
The file manager is responsible for loading and saving files for the project.
it must be initialized to the data directory that stores all the config files for this project.
every file reference should be relative to the _dataDir.
*/

class FileManager : public Serializable
 {
 private:
	 GsString _dataDir;
	 /*the relative path to the directory containing all the data*/
	
 public :
	 //get the root directory of the FileManager
	GsString getDataDir();
    FileManager ( GsString& file);

	//delete a file with system command
	void deleteFile(const GsString& s );
	//create a directory with system command
	void makeDirectory(const GsString& dirName);
	//create a directory newDirectory within directory dir
	void makeSubDirectory(const GsString& dir ,const GsString& newDirectory);
	//check if a directory parentD (ex ../data/character/) contains another directory searchD
	bool subDirectoryExists(const GsString& parentD,const GsString& searchD );
	//remove the directory with system command
	void deleteDirectory(const GsString& s );
	//check if a directory is already made
	bool directoryExists(const GsString& searchD );
	//get all the files in directory dir with optional extension (ex "png")
	void getFiles(const GsString& dir, GsStrings& files,const GsString& ext = "" ); 	
	//fill dirs with all the directories in dir
	void getDirectories(const GsString& dir, GsStrings& dirs );
	//remove the data directory prefix ../data/ from a string
	void removeDataDir(GsStrings& files );
};
void parent_directory(GsString& s);
void directory_short_name(GsString& s);


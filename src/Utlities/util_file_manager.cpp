


#include "util_file_manager.h"


#include <gsim/gs_scandir.h>
#include <stdlib.h>
#include <direct.h>

FileManager::FileManager (GsString& dir) : Serializable("FileManager")
{
	 _dataDir = dir;
	_dataDir<<SLASH;
	#ifdef VERIFY_PARAMETERS
		verifyParameters();
	#endif  

}
GsString FileManager::getDataDir()
{
	return _dataDir;
}
void FileManager::deleteFile(const GsString& s )
{
	GsString n = "del /q ";
	n<<s;
	//phout<<"sending command "<<n<<gsnl;
	system(n);
}
void FileManager::deleteDirectory(const GsString& s )
{
	phout<<"deleting directory "<<s<<gsnl;
	GsString cmd = "rd /s /q ";
	cmd<<s;
	system(cmd);
}
void FileManager::makeSubDirectory(const GsString& directory ,const GsString& newDirectory)
{
	GsString dir = directory;
	dir<<newDirectory<<SLASH;
	makeDirectory(dir);
}
void parent_directory(GsString& s)
{
	//if there is a file it will remove that and return its directory name
	//if it is a directory it will be its parent directory
	for (int i=s.len()-2;i>1;i--)
	{
		if(s.get(i)=='\\')
		{
			s.substring(0,i);
			return;
		}
	}
}
void directory_short_name(GsString& s)
{
	if(s.get(s.len()-1)=='\\')
	{
		//this is a directory so just remove the last slash and remove the path
		s.substring(0,s.len()-2);
		remove_path(s);
		return;
	}
	else
	{
		//this is a file so remove filename and path;
		GsString filename = s;
		remove_path(filename);
		s.remove(s.len()-filename.len()-1,filename.len()+1);
		remove_path(s);
		return;

	}

}
void FileManager::makeDirectory(const GsString& directory)
{
	GsString dirname = getDataDir();
	dirname<<directory;
	if(!directoryExists(dirname))
	{
		GsString cmd = "mkdir ";
		cmd<<dirname;
		//gsout<<"about to send command ["<<cmd<<"]"<<gsnl;	
		//system("mkdir ..\data\test\p1\") ;
		//		gsout<<"ret is "<<ret<<"  after sent command ["<<cmd<<"]"<<gsnl;
		//int ret =	system(cmd); 
		mkdir(dirname);
			//mkdir()
		//after this point all the calls to gsout are corrupted
		//they have strange characters and are all on one line, seeming to overwrite the previous line
		//gsout<<"ret is "<<ret<<"  after sent command ["<<cmd<<"]"<<gsnl;	
	}
	else
	{
		//phout<<"directory "<<directory<<" already exists\n";
	}
}
bool FileManager::directoryExists(const GsString& searchD )
{
	GsString parentD = searchD;
	parent_directory(parentD);
	GsString shortName = searchD;
	directory_short_name(shortName);
	return subDirectoryExists(parentD,shortName);
}
bool FileManager::subDirectoryExists(const GsString& parentD,const GsString& searchD )
{
	GsStrings files;
	GsStrings subdir;
	GsStrings ext;
	gs_scandir(parentD,subdir,files,ext);
	bool found = false;
	GsString searchDir = parentD;
	searchDir<<searchD;
	for (int i=0;i<subdir.size();i++)
	{
		if(subdir[i] == searchDir)
		{
			return true;
		}
	}
	return false;
}


void FileManager::getFiles(const GsString& dir, GsStrings& files,const GsString& ext )
{
	GsString dirname = getDataDir();
	dirname<<dir;
	GsStrings exts;
	exts.push(ext);
	GsStrings sdir;
	gs_scandir(dirname,sdir,files,exts);

	removeDataDir(files);
	
}


void FileManager::getDirectories(const GsString& dir, GsStrings& dirs )
{
	GsString dirname = getDataDir();
	dirname<<dir;
	GsStrings exts;
	GsStrings files;
	gs_scandir(dirname,dirs,files,exts);
}

void FileManager::removeDataDir( GsStrings& files )
{
	for (int i=0;i<files.size();i++)
	{
		GsString name = files[i];
		if(GsString::compare(name,getDataDir())>0)
		{
			name.substring( getDataDir().len(), name.len());
			files.set(i,  name) ;
			//gsout<<"converting name to "<<name<<gsnl;
		}
		else
		{
			gsout<<"string "<<name<<" doesn't start with "<<getDataDir()<<gsnl;
		}
	}
}

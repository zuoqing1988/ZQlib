#ifndef _SAMPLE_GRID_DEFORMATION_SCENE_DESCRIPTION_H_
#define _SAMPLE_GRID_DEFORMATION_SCENE_DESCRIPTION_H_
#pragma once

template<class T>
class SampleGridDeformationSceneDescription
{
public:
	SampleGridDeformationSceneDescription();
	~SampleGridDeformationSceneDescription();

private:
	int width;
	int height;
	T* init_coord;
	bool* nouseful_flag;
	bool* fixed_flag;

public:
	bool LoadFromFile(const char* file);
	bool LoadFromArgs(const int argc, const char** argv);

	const int GetWidth() const {return width;}
	const int GetHeight() const {return height;}
	const bool* GetNousefulFlag() const {return nouseful_flag;}
	const bool* GetFixedFlag() const {return fixed_flag;}
	const T* GetInitCoord() const {return init_coord;}


private:
	void _clear();
};


template<class T>
SampleGridDeformationSceneDescription<T>::SampleGridDeformationSceneDescription()
{
	width = 0;
	height = 0;
	nouseful_flag = 0;
	fixed_flag = 0;
	init_coord = 0;
}

template<class T>
SampleGridDeformationSceneDescription<T>::~SampleGridDeformationSceneDescription()
{
	_clear();
}

template<class T>
bool SampleGridDeformationSceneDescription<T>::LoadFromFile(const char* file)
{
	FILE* in = fopen(file,"r");
	if(in == 0)
		return false;

	std::vector<char*> args;
	char buf[2000];
	do 
	{
		strcpy(buf,"");
		fscanf(in,"%s",buf);
		if(buf[0] == '\0')
			break;
		int len = strlen(buf);
		char* tmp = new char[len+1];
		strcpy(tmp,buf);
		args.push_back(tmp);
	} while (true);
	fclose(in);

	int argc = args.size();
	if(argc == 0)
		return false;

	bool ret = LoadFromArgs(argc,(const char**)&args[0]);
	for(int i = 0;i < argc;i++)
		delete []args[i];

	return ret;
}

template<class T>
bool SampleGridDeformationSceneDescription<T>::LoadFromArgs(const int argc, const char** argv)
{
	_clear();
	for(int k = 0;k < argc;k++)
	{
		if(_strcmpi(argv[k],"resolution") == 0)
		{
			k++;
			if(k >= argc)
			{
				printf("the value of resolution:x ?\n");
				return false;
			}
			width = atoi(argv[k]);
			k++;
			if(k >= argc)
			{
				printf("the value of resolution:y ?\n");
				return false;
			}
			height = atoi(argv[k]);
			nouseful_flag = new bool[width*height];
			fixed_flag = new bool[width*height];
			init_coord = new T[width*height*2];
			if(nouseful_flag == 0 || fixed_flag == 0 || init_coord == 0)
			{
				return false;
			}
			memset(nouseful_flag,0,sizeof(bool)*width*height);
			memset(fixed_flag,0,sizeof(bool)*width*height);
			memset(init_coord,0,sizeof(T)*width*height*2);
		}
		else if(_strcmpi(argv[k],"nouseful") == 0)
		{
			int coord_x;
			int coord_y;
			k++;
			if(k >= argc)
			{
				printf("the value of nouseful:coord_x ?\n");
				return false;
			}
			coord_x = atoi(argv[k]);
			k++;
			if(k >= argc)
			{
				printf("the value of nouseful:coord_y ?\n");
				return false;
			}
			coord_y = atoi(argv[k]);
			if(coord_x < 0 || coord_x >= width || coord_y < 0 || coord_y >= height)
			{
				printf("invalid nouseful point: <%d,%d>\n",coord_x,coord_y);
				return false;
			}
			nouseful_flag[coord_y*width+coord_x] = true;
		}
		else if(_strcmpi(argv[k],"fixed") == 0)
		{
			int coord_x;
			int coord_y;
			float pos_x;
			float pos_y;

			k++;
			if(k >= argc)
			{
				printf("the value of fixed:coord_x ?\n");
				return false;
			}
			coord_x = atoi(argv[k]);
			k++;
			if(k >= argc)
			{
				printf("the value of fixed:coord_y ?\n");
				return false;
			}
			coord_y = atoi(argv[k]);
			k++;
			if(k >= argc)
			{
				printf("the value of fixed:pos_x ?\n");
				return false;
			}
			pos_x = atof(argv[k]);
			k++;
			if(k >= argc)
			{
				printf("the value of fixed:pos_y ?\n");
				return false;
			}
			pos_y = atof(argv[k]);
			if(coord_x < 0 || coord_x >= width || coord_y < 0 || coord_y >= height)
			{
				printf("invalid fixed point: <%d,%d,%f%f>\n",coord_x,coord_y,pos_x,pos_y);
				return false;
			}
			fixed_flag[coord_y*width+coord_x] = true;
			init_coord[(coord_y*width+coord_x)*2+0] = pos_x;
			init_coord[(coord_y*width+coord_x)*2+1] = pos_y;
		}
		else
		{
			printf("invalid parameter: %s\n",argv[k]);
		}
	}
	return true;
}

template<class T>
void SampleGridDeformationSceneDescription<T>::_clear()
{
	width = 0;
	height = 0;
	if(nouseful_flag)
	{
		delete []nouseful_flag;
		nouseful_flag = 0;
	}
	if(fixed_flag)
	{
		delete []fixed_flag;
		fixed_flag = 0;
	}
	if(init_coord)
	{
		delete []init_coord;
		init_coord = 0;
	}
}

#endif

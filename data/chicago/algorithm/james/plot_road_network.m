% James code
clear;
cd 'C:\Users\DELL\Documents\PhD\OtherApproaches\gis12_mapinference\gis12_mapinference\chicago\campus_set_results\'

[fnames,path]=uigetfile('*.txt','...select file!','MultiSelect','on');
hold on;

wf=['chicago_james_edges.txt'];
wff=['chicago_james_vertices.txt'];
fileID = fopen(wf,'w');
fileIDD = fopen(wff,'w');
w=1;


for k=1:length(fnames(:,1))
    mat=load([path,fnames(1,:)]);
    sm=size(mat);
    len=sm(1);
    [x,y,utmzone] = wgs2utm(mat(:,1),mat(:,2));
    for g=1:2:len
        line(1,1)=mat(g,2);
        line(1,2)=mat(g,1);
        line(2,1)=mat(g+1,2);
        line(2,2)=mat(g+1,1);
        plot(line(:,1),line(:,2),'-r','linewidth',2);
		
		if g==length(mat(:,1))-1
		fprintf(fileID,'%d,%d,%d\n',w,g,g+1);
		else
		fprintf(fileID,'%d,%d,%d\n',w,g,g+2);
		end
		
		fprintf(fileIDD,'%d,%f,%f\n',g,x(g),y(g));
		if g==length(mat(:,1))-1  
			fprintf(fileIDD,'%d,%f,%f\n',g+1,x(g+1),y(g+1));
		end
		w=w+1;	
    end
end


fclose(fileID);
fclose(fileIDD);

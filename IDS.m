clear all;
% MapInit=[1 0 0 0;0 0 0 0;0 0 2 0;0 4 3 0];
% MapInit=[0 0 0 0;0 1 0 0;0 2 0 4;0 3 0 0];
% MapInit=[0 0 0 0;0 1 0 0;0 2 0 0;0 4 3 0];
MapInit=[1 0 0 0;0 0 0 0;4 0 0 2;0 0 0 3];
goal=[0 0 0 0;0 1 0 0;0 2 0 0;0 3 0 4];
Map=MapInit;
Maptemp=Map;
depth=zeros(100,1);
breth=zeros(100,1);
stack=cell(50,1);
set_depth=20; %set the depth limit(it maybe better to let the program find itself)
visited=0;
stack{1}=MapInit;
path=zeros(1000000,7);
direction=[-1,0;1,0;0,-1;0,1];
bip=[4,4];%blockinitposition
temp=bip;
j=0;
m=0;
score=0;
%% Main code
while score==0;
    m=m+1;
    Map=stack{m};
    Maptemp=Map;
    direction_index=randperm(4);
    for l=1:4
    b(l,:)=direction(direction_index(:,l),:);
    end
    [row,col]=find(Map==4);
    bip=[row,col];
    temp=bip;
    for n=1:4
        bip=bip+b(n,:);
        moverror=find(bip<1|bip>4);
        visited=find(path(m,1:4)==direction_index(n));
        if path(m,7)==4;
           path(m:m+1,1:4)=0;
           path(m:m+1,7)=0;
           m=m-2;
%            if m<1
%               return
%            end
           break
        else
            if (~isempty(moverror))
                bip=temp;
                path(m,7)=path(m,7)+1;
                path(m,path(m,7))=direction_index(n);
            elseif visited~=0
                bip=temp;
            else
                Maptemp(bip(1,1),bip(1,2))=Map(temp(1,1),temp(1,2));
                Maptemp(temp(1,1),temp(1,2))=Map(bip(1,1),bip(1,2));
                stack{m+1}=Maptemp;
                t1=find(-b(n,1)==direction(:,1));
                t2=find(-b(n,2)==direction(:,2));
                t=intersect(t1,t2);
                path(m,7)=path(m,7)+1;
                path(m,path(m,7))=direction_index(n);
                path(m+1,7)=path(m+1,7)+1;
                path(m+1,path(m+1,7))=t;
                path(m+1,6)=m;
                j=j+1;
                break
            end
        end
    end
    %% Tell have the research meet the limit
    if set_depth==path(m+1,6) 
        if goal==stack{m+1};%if the state meet the goal
            steps=m;
            score=1;
            return
        else
            score=0;
            m=m-1;
        end
    else %if the state haven't meet the limit 
        if goal==stack{m+1};
            steps=m+1;
            score=1;
            return
        else % keep running
            score=0; 
        end
    end
  end
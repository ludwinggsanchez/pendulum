clc;
close all;
% clear;
% ar = arduino('COM4');
Fs=10;
N=100;
referencia=zeros(N,1);
salidateo=zeros(N,1);
salidap=zeros(N,1);
t=linspace(0,(N-1)/Fs,N);
%l1=line(nan,nan,'Color','r','LineWidth',2);
%l2=line(nan,nan,'Color','b','LineWidth',1);
l3=line(nan,nan,'Color','g','LineWidth',2);
ylim([-91 91])
%ylim([0 5])
grid
%title('Ángulos de entrada y salida en el tiempo')
title('Ángulo de salida en el tiempo')
xlabel('$\frac{t}{\mathrm{s}}$','Interpreter','latex','FontSize',20);
ylabel('$\frac{\theta_{out}}{\mathrm{grados}}$','Interpreter','latex','FontSize',20);
%legend('Señal de entrada','Señal de salida')
legend('Señal de salida')
Stop=1;
vin=zeros(N,1);
vout=zeros(N,1);
% G=tf([0 7.55],[1 32 11.806]);
% Hd=c2d(G,1/Fs);
% a=Hd.den(1);
% a1=a(2);
% a2=a(end);
% b=Hd.num(1);
% b1=b(2);
% b2=b(end);
uicontrol('Style','Pushbutton','String','Parar','Position',[10,5,60,20],...
    'Callback','Stop=0;')

tic
while Stop
      if toc>1/Fs
          tic
          referencia(1:end-1)=referencia(2:end);
          salidateo(1:end-1)=salidateo(2:end);
          salidap(1:end-1)=salidap(2:end);
          referencia(end)=(37.217)*(ar.analogRead(1)*5/1023)-92.075;
          %referencia(end)=ar.analogRead(1)*5/1023;
          salidap(end)=(37.217)*(ar.analogRead(6)*5/1023)-92.075;
          %salidateo(end)=-a1*salidateo(end-1)-a2*salidateo(end-2)+b1*referencia(end-1)+b2*referencia(end-2);
          %set(l1,'XData',t,'YData',referencia);
          %set(l2,'XData',t,'YData',salidateo);
          set(l3,'XData',t,'YData',salidap);
          drawnow
      end 
end        
          
          



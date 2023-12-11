function [] = plotter(ii,t,Ej,Yj,Uj,Rj,U)

  figure(1)

  % Plot the error Ej of the current itteration
  subplot(1,3,1);
  plot(t,Ej,'LineWidth',1.5);
  title('Error, Ej','FontSize',16);
  ylabel('Error Response','FontSize',16);
  ylim([-5 5])

  % Plot the input Uj of the current itteration
  subplot(1,3,2);
  plot(t,Uj,t,U,'-k','LineWidth',1.5);
  title({['Iteration: ', num2str(ii)],'Input, Uj'},'FontSize',16);
  xlabel('Time (s)','FontSize',16);
  ylabel('Input Response','FontSize',16);
  ylim([0 7])

  % Plot the output Yj of the current itteration
  subplot(1,3,3);
  plot(t,Yj,t,Rj,'-k','LineWidth',1.5);
  title('Output, Yj','FontSize',16);
  ylabel('Output Response','FontSize',16);
  ylim([0 7])

  pause(0.5);

end
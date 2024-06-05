function h = personal_plot3(x, y1, y2, y3, x_label_latex, y_label_latex, y1_legend_latex, y2_legend_latex, y3_legend_latex, pdf_name)

set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
lw_r = 2;
lw_b = 1;
lw_g = 1;

h = figure('Renderer', 'painters', 'Position', [10 10 900 600]);
removeToolbarExplorationButtons(h)

plot(x, y1, 'k-', 'Linewidth', lw_r ,'Color', [1, 0, 0]);
hold on
plot(x, y2, 'k-', 'Linewidth', lw_b ,'Color', [0, 0, 1]);
hold on
plot(x, y3, 'k-', 'Linewidth', lw_g ,'Color', [0, 1, 0]);

legend(y1_legend_latex, y2_legend_latex, y3_legend_latex);
xlabel(x_label_latex)
ylabel(y_label_latex)
xlim([x(1) x(end)])
set(gca, 'FontSize',18);
grid on
box on
set(gcf,'color','w');
legend('Location','northoutside','Orientation','horizontal')
exportgraphics(h, pdf_name);

end

# Execute with something like: 
#      sudo R --vanilla -f flow_analysis.R --args /home/pi/logs/log_shot-2016-Jan-09-1305.json

args <-commandArgs(TRUE)
library(rjson)
d <- fromJSON(file = args)

# d <- fromJSON(file="~/Desktop/log_shot2016-01-10-0055.json")

df <- data.frame(weight = d$weight, time = d$time, t0 = d$t0, t1 = d$t1, t2 = d$t2, flow_per_second = d$flow_per_second, predicted_end_time = d$predicted_end_time, pump_power = d$pump_power*10)
df$time <- df$time - d$start
df <- df[df$time >= 0,]

dw <- data.frame(weight = d$full_weight_series, time = d$full_weight_series_time)
dw$time <- dw$time - d$start
dw <- dw[dw$time >= 0,]


which_i <- function(y){
	which(abs(dw$weight - y) == min(abs(dw$weight - y)))[1]
}

###########################
# 4-Graph Layout:         #
#    - Weight             #
#    - Flow rate          #
#    - Predicted end      #
#    - Pump pressure      #
###########################


xmax <- max(d$time)-d$start + .2
# rows <- df$time >= 14 & df$time < d$end - d$start - .1
rows <- df$flow_per_second > 0 & df$time < d$end - d$start - .1

pred_min <- min(min(df$predicted_end_time[rows] - 1), 28)
pred_max <- min(max(df$predicted_end_time[rows] + 1), 55)

flow_max <- max(max(df$flow_per_second) + 0.1, 3.5)

library(cowplot)

name <- substr(args, 15, nchar(args)-5)
graph_filename <- paste("/home/pi/logs/graph-", name, ".pdf", sep = "")

pdf(height = 8, width = 5, file = graph_filename)

# quartz(height = 8, width = 5)

###########################
# Weight vs. Time         #
###########################

g1 <- ggplot() +
		geom_hline(yintercept = d$max_weight) + geom_vline(xintercept = d$end - d$start) +
		geom_segment(data = df[rows,], aes(x = time, y = weight, xend = predicted_end_time, yend = d$max_weight), col = "red", alpha = I(1/10), show.legend = FALSE) +
		# geom_point(data = df[rows,], aes(x=time, y=weight), col = "red", alpha = I(1/10)) +
		geom_line( data = dw, aes(x=time, y=weight)) + 
		labs(title = paste(round(df$weight[nrow(df)],0), "g. in", round(d$end - d$start,0), "s."), y="Weight in grams", x= element_blank()) +
		coord_cartesian(xlim = c(-.8,xmax), ylim = c(min(dw$weight)-.5, max(dw$weight) + 1)) +
		annotate("segment", x=dw[which_i(5),]$time, y=dw[which_i(5),]$weight, xend=dw[which_i(5),]$time, yend=-10, linetype = 3) +
		annotate("segment", x=dw[which_i(10),]$time, y=dw[which_i(10),]$weight, xend=dw[which_i(10),]$time, yend=-10, linetype = 3) +
		annotate("segment", x=dw[which_i(20),]$time, y=dw[which_i(20),]$weight, xend=dw[which_i(20),]$time, yend=-10, linetype = 3) +
		geom_text(data = dw[c(which_i(5), which_i(10), which_i(20)),], aes(x=time, y=weight, label=paste(round(time,1), " s.", sep = "")), vjust = -1.8, size = 3)+
		theme(plot.margin = unit(c(1, 1, 0, 1), "lines"))  

###########################
# Flow rate vs. Time      #
###########################

g2 <- ggplot(df[rows,], aes(x = time, y=flow_per_second)) + 
		geom_line(color = "red2") + geom_point(color = "red2") + 
		labs(y="Flow rate", x = element_blank()) +
		geom_vline(xintercept = d$end - d$start) + 
		coord_cartesian(xlim = c(-.8,xmax), ylim = c(-0.05, min(flow_max, 3.5))) +
		theme(plot.margin = unit(c(0.5, 1, 0, 1), "lines"))

###############################
# Predicted end vs. Time      #
###############################

g3 <- ggplot(df[rows,], aes(x = time, y=predicted_end_time)) + 
		geom_ribbon(aes(ymin = 0, ymax = t2, x = time), fill = "blue", alpha = I(.1))+
		geom_ribbon(aes(ymin = t2, ymax = t1, x = time), fill = "yellow", alpha = I(.1))+
		geom_ribbon(aes(ymin = t1, ymax = t0, x = time), fill = "green", alpha = I(.05))+
		geom_ribbon(aes(ymin = t0, ymax = 100, x = time), fill = "red", alpha = I(.1))+
		geom_line(color = "darkgreen") + geom_point(color = "darkgreen") + 
		labs(y="Predicted end time", x = element_blank()) + 
		geom_vline(xintercept = d$end - d$start) + 
		coord_cartesian(xlim = c(-.8, xmax), ylim = c(pred_min, pred_max)) +
		theme(plot.margin = unit(c(0.5, 1, 0, 1), "lines"))

###########################
# Pump power vs. Time     #
###########################

g4 <- ggplot(df, aes(x = time, y=pump_power)) + 
		geom_step(color = "royalblue") +
		geom_vline(xintercept = d$end - d$start) +
		labs(y="Pump power (%)", x=paste("Time in seconds\n[sent ",format(Sys.time(), "%A, %B %d, %Y %H:%M"),"]",sep = "")) +
		coord_cartesian(xlim = c(-.8, xmax), ylim = c(-5, 105)) +
		scale_y_continuous(breaks = seq(0,100,20), minor_breaks = NULL) +
		theme(plot.margin = unit(c(0.5, 1, 2, 1), "lines"))

theme_set(theme_gray())
plot_grid(g1, g2, g3, g4, ncol=1, align = "v", rel_heights = c(1, .6, .6, .85))
dev.off()

system(paste("sudo python /home/pi/send_email.py", name))
# Makefile to build graphs for all UAL eCAR "applications", ROS nodes, etc.
# J.L.Blanco 2017
.PHONY = clean

DOT_PNG := $(patsubst %.doth,%.png,$(wildcard ../*/*.doth))
APPS_PNG := $(patsubst %.app,%.png,$(wildcard *.app))

all: $(DOT_PNG) $(APPS_PNG)

clean:
	rm $(DOT_PNG) $(APPS_PNG) 2>/dev/null || true

%.png: %.doth
	@echo "== BUILDING: $<"
	@echo "digraph ros_node {" > $<.dot
	@cat $< >> $<.dot
	@echo "}" >> $<.dot
	dot -T png $<.dot -o $@

%.png: %.app
	@echo "== BUILDING: $<"
	@echo "digraph ros_launch {" > $<.dot
	@cpp $< >> $<.dot
	@echo "}" >> $<.dot
	dot -T png $<.dot -o $@

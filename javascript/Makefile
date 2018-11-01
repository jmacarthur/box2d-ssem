# Makefile is used just to create box2d.min.js.

compressed: library_list
	rm -f box2d.min.js
	for i in `cat library_list`; do cat $$i >> box2d.js; done
	uglifyjs box2d.js > box2d.min.js
	rm box2d.js

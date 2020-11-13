(function(dataString) {
	var data = JSON.parse(dataString);
	return data.playingSequence ? 1 : 0;
})(input)
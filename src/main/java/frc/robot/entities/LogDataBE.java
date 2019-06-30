package frc.robot.entities;

import java.util.ArrayList;
import java.util.List;

// This is a "data entity" class that hold the data to logged.
// Subsystem classes use the Add method to add data in their UpdateLogData method as Name/Value pairs
// Internally, this class holds the Names & Values in 2 arrays
// Therefore this class does not need to be changed to support addl data to be logged, it grows dynamically
public class LogDataBE {

	// define class level working variables
	private List<String> _names;
	private List<String> _values;
	private long _logStartTimeStampinMS;
	private long _logDataTimeStampinMS;
	private String _markerName = null;
	private long _logQueueDepth;

	// constructor(s)
	public LogDataBE(long logDataTimeStampinMS) {
		_names = new ArrayList<String>();
		_values = new ArrayList<String>();

		_logDataTimeStampinMS = logDataTimeStampinMS;
	}

	public boolean get_isEmpty()
	{
		return _names.isEmpty() || _values.isEmpty();
	}

	public long get_logDataTimeStampinMS()
	{
		return _logDataTimeStampinMS;
	}

	public void set_logStartTimeStampinMS(long logStartTimeStampinMS)
	{
		_logStartTimeStampinMS = logStartTimeStampinMS;
	}

	public void set_marker(String markerName)
	{
		_markerName = markerName;
	}

	public String get_marker()
	{
		return _markerName;
	}

	public void set_logQueueDepth(long logQueueDepth)
	{
		_logQueueDepth = logQueueDepth;
	}

	public long get_logQueueDepth()
	{
		return _logQueueDepth;
	}

	/** Add a field name/value pair to the log record */
	public void AddData(String name, String value) {
		_names.add(name);
		_values.add(value);
	}
	
	/** Build a TSV (tab separated value) string for the header row */
	public String BuildTSVHeader() {
		List<String> names = new ArrayList<String>(_names);
		return BuildTSVString(names);
	}

	/** Build a TSV (tab separated value) string for a data row */
	public String BuildTSVData() {
		List<String> values = new ArrayList<String>(_values);
		return BuildTSVString(values);
	}
	
	/** Build a TSV string from a List<string> */
	private String BuildTSVString(List<String> myList) {
		// create stringbuilder with a larger initial capacity to improve perf
		StringBuilder sb = new StringBuilder(550);
		
		for(String item : myList) {
			// add the item + a tab character
			sb.append(item + "\t");
		}

		// add trailing crlf
		sb.append("\r\n");
		
		return sb.toString();
	}
}
var ArrayList = Java.type("java.util.ArrayList");

// Get the arguments string from this ScriptRun stage.
// Convention: first token in Arguments is the source stage name.
// Example: Arguments = "findPocketRect"
var argsString = stage.getArguments(); // ScriptRun property
argsString = argsString == null ? "" : argsString.trim();

if (argsString.length === 0) {
  print("rectAsList: No arguments provided. Expected: <sourceStageName>\n");
  return new ArrayList(); // empty list -> clean failure
}

// Take the first whitespace-separated token as the stage name
var sourceStageName = argsString.split(/\s+/)[0];

print("rectAsList: Using source stage '" + sourceStageName + "'\n");

// --------------------------------------------------------------------
// Get the RotatedRect model from the named stage.
// Depending on your OpenPnP version, either of these may work:
//
//  1) If a 'model' map is provided to the script:
//       var rect = model[sourceStageName];
//
//  2) If not, fall back to pipeline.getModel(sourceStageName);
//
// I’ll try #1 first, then #2 as a fallback.
// --------------------------------------------------------------------

var rect = null;

// Try model[...] first if available
try {
  if (typeof model !== "undefined" && model[sourceStageName] !== undefined) {
    rect = model[sourceStageName];
  }
} catch (e) {
  // ignore, we'll try pipeline below
}

if (rect == null) {
  try {
    // Typical CvPipeline API: getModel(String stageName)
    rect = pipeline.getModel(sourceStageName);
  } catch (e) {
    print(
      "rectAsList: Could not read model from stage '" +
        sourceStageName +
        "': " +
        e +
        "\n"
    );
  }
}

if (rect == null) {
  print(
    "rectAsList: No model found for stage '" +
      sourceStageName +
      "'. Returning empty list.\n"
  );
  return new ArrayList();
}

// Wrap the single RotatedRect into a List<RotatedRect>
var list = new ArrayList();
list.add(rect);

print(
  "rectAsList: Wrapped RotatedRect from '" +
    sourceStageName +
    "' into list: " +
    rect +
    "\n"
);

return list;

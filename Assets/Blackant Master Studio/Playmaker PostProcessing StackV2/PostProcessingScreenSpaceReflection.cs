using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

namespace HutongGames.PlayMaker.Actions
{

	[ActionCategory("Post Processing Stack V2")]
	[Tooltip("Modify Screen Space Reflection During Runtime.")]
	public class PostProcessingScreenSpaceReflection : FsmStateAction
	{

		#region public variables
				
		[RequiredField]
		[ObjectType(typeof(PostProcessProfile))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmObject ObjectProfile;

		[Tooltip("Activate or deactivate Screen Space Reflections Layer")]
		public FsmBool ActiveLayer;
		[Tooltip("Activate or deactivate Screen Space Reflections Effect")]
		public FsmBool ActiveEffect;

		[Tooltip("Activate Shutter Angle parameter")]
		public FsmBool PresetActive;
		[Tooltip("Scale Value")]
		public ParameterOverride<ScreenSpaceReflectionPreset> Preset;

		[Tooltip("Activate Shutter Angle parameter")]
		public FsmBool MaxMarchDistanceActive;
		[Tooltip("Scale Value")]
		public FsmFloat MaxMarchDistance;

		[Tooltip("Activate Shutter Angle parameter")]
		public FsmBool DistanceFadeActive;
		[HasFloatSlider(0f, 1f)]
		[Tooltip("Scale Value")]
		public FsmFloat DistanceFade;

		[Tooltip("Activate Shutter Angle parameter")]
		public FsmBool VignetteActive;
		[HasFloatSlider(0f, 1f)]
		[Tooltip("Scale Value")]
		public FsmFloat Vignette;

		[Tooltip("Activate Maximum Iteration")]
		public FsmBool MaxIterationCountActive;
		[HasFloatSlider(0, 256)]
		[Tooltip("Scale Value")]
		public FsmInt MaxIterationCount;
		[Tooltip("Activate Thickness")]
		public FsmBool ThicknessActive;
		[HasFloatSlider(1, 64)]
		[Tooltip("Scale Value")]
		public FsmInt Thickness;


		[Tooltip("Repeat every frame.")]
		public bool everyFrame;
		#endregion

		#region Private variables
		private ScreenSpaceReflections ScreenReflectionsLayer;
		private PostProcessProfile UsedProfile;	
		#endregion

		public override void Reset()
		{
			ActiveLayer= true;
			ActiveEffect = true;
			ObjectProfile = null;
			PresetActive = true;
			//Preset = 0;
			MaxMarchDistanceActive = true;
			MaxMarchDistance = 100f;
			DistanceFadeActive = true;
			DistanceFade = 0.5f;
			VignetteActive = true;
			Vignette = 0.5f;
			everyFrame = false;
			
		}

		// Code that runs on entering the state.
		public override void OnEnter()
		{
			SetScreenReflections();
			if(!everyFrame)
			{
				Finish();
			}
		}

		// Code that runs every frame.
		public override void OnUpdate()
		{
			SetScreenReflections();
		}

		void SetScreenReflections()
		{
						
				UsedProfile = ObjectProfile.Value as PostProcessProfile;
				UsedProfile.TryGetSettings(out ScreenReflectionsLayer);
				if(ObjectProfile != null & UsedProfile.HasSettings<ScreenSpaceReflections>())
				{
					if(ScreenReflectionsLayer!=null)
					{
						ScreenReflectionsLayer.active = ActiveLayer.Value; //Active AO Layer
						ScreenReflectionsLayer.enabled.value = ActiveEffect.Value; // Active AO Effect
						ScreenReflectionsLayer.preset.overrideState = PresetActive.Value;//Activate preset
						if(PresetActive.Value)
						{ 	
							ScreenReflectionsLayer.preset.value = Preset; // preset 
						}
						ScreenReflectionsLayer.maximumIterationCount.overrideState = MaxIterationCountActive.Value;
						if (MaxIterationCountActive.Value)
						{
							ScreenReflectionsLayer.maximumIterationCount.value = MaxIterationCount.Value;
						}
						ScreenReflectionsLayer.thickness.overrideState = ThicknessActive.Value;
						if (ThicknessActive.Value)
						{
							ScreenReflectionsLayer.thickness.value = Thickness.Value;
						}
						ScreenReflectionsLayer.maximumMarchDistance.overrideState = MaxMarchDistanceActive.Value;//max march distance active
						if(MaxMarchDistanceActive.Value)
						{ 	
							ScreenReflectionsLayer.maximumMarchDistance.value = MaxMarchDistance.Value; // max march distance
						}
						ScreenReflectionsLayer.distanceFade.overrideState = DistanceFadeActive.Value;//distance fade active
						if(DistanceFadeActive.Value)
						{ 	
							ScreenReflectionsLayer.distanceFade.value = DistanceFade.Value; // distance fade
						}
						ScreenReflectionsLayer.vignette.overrideState = VignetteActive.Value;//vignette active
						if(VignetteActive.Value)
						{ 	
							ScreenReflectionsLayer.vignette.value = Vignette.Value; // vignette
						}
					}
				}
			}
		}
}



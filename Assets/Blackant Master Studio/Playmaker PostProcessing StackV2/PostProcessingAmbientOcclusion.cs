using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

namespace HutongGames.PlayMaker.Actions
{

	[ActionCategory("Post Processing Stack V2")]
	[Tooltip("Modify Ambient Occlusion Layer Settings")]
	public class PostProcessingAmbientOcclusion : FsmStateAction
	{	
		#region public variables
				
		[RequiredField]
		[ObjectType(typeof(PostProcessProfile))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmObject ObjectProfile;

		[Tooltip("Activate or deactivate AO Layer")]
		public FsmBool ActiveLayer;
		[Tooltip("Activate or deactivate AO Effect")]
		public FsmBool ActiveEffect;

		[Tooltip("Activate Mode Selection")]
		public FsmBool ActiveMode;
		[Tooltip("Set the AO Mode")]
		public ParameterOverride<AmbientOcclusionMode> AoType;

		[Tooltip("Activate Quality modification")]
		public FsmBool ActiveQuality;
		[Tooltip("Set the Quality")]
		public AmbientOcclusionQuality AOQuality;
		
		[Tooltip("Activate Intensity modification")]
		public FsmBool ActiveIntensity;
		[HasFloatSlider(0f, 4f)]
		[Tooltip("Intensity of Occlusion Shadow")]
		public FsmFloat Intensity;

		[Tooltip("Activate Radius modification")]
		public FsmBool ActiveRadius;
		[Tooltip("Radius of Occlusion shadow")]
		public FsmFloat Radius;
		
		[Tooltip("Activate Color modification")]
		public FsmBool ActivateColor;
		[Tooltip("Set the Color of the AO")]
		public FsmColor AOColor;

		[Tooltip("Activate Ambient Only modification")]
		public FsmBool ActivateAoOnly;
		[Tooltip("Is Ambient Occlusion Only ? (Defered + HDR)")]
		public FsmBool AmbientOnly;

		[Tooltip("Activate Thickness modification")]
		public FsmBool ActivateThickness;
		[HasFloatSlider(1f, 10f)]
		[Tooltip("Thickness of Effect")]
		public FsmFloat ThicknessModifier;	
		
		[Tooltip("Repeat every frame.")]
		public bool everyFrame;
		#endregion

		#region Private variables
		private AmbientOcclusion AoLayer;
		private PostProcessProfile UsedProfile; 		
		#endregion

        public override void Reset()
		{
			ActiveLayer= true;
			ActiveEffect = true;
			ObjectProfile = null;
			Intensity = 0f;
			Radius = 0f;
			AoType = null;
			AOQuality = 0;
			AOColor = Color.black;
			AmbientOnly = false;
			ThicknessModifier = 1f;
			everyFrame = false;
			AoLayer = null;
			ActiveMode = true;
			ActiveIntensity =true;
			ActiveRadius = true;
			ActiveQuality = true;
			ActivateColor =true;
			ActivateAoOnly = true;
			ActivateThickness = true;
			
		}

		// Code that runs on entering the state.
		public override void OnEnter()
		{
			SetSettings();
			if(!everyFrame)
			{
				Finish();
			}
		}

		// Code that runs every frame.
		public override void OnUpdate()
		{
			SetSettings();
		}

		void SetSettings()
		{
						
				UsedProfile = ObjectProfile.Value as PostProcessProfile;
				UsedProfile.TryGetSettings(out AoLayer);
				if(ObjectProfile != null & UsedProfile.HasSettings<AmbientOcclusion>())
				{
					if(AoLayer!=null)
					{	
						AoLayer.active = ActiveLayer.Value; //Active AO Layer
						AoLayer.enabled.value = ActiveEffect.Value; // Active AO Effect
						AoLayer.mode.overrideState=ActiveMode.Value;//Activate Mode
						if(ActiveMode.Value)
						{ 	
							AoLayer.mode.value = AoType; // AO Mode selection
						}
						AoLayer.intensity.overrideState=ActiveIntensity.Value; //Activate Intensity modification
						if(ActiveIntensity.Value)
						{
							AoLayer.intensity.value = Intensity.Value; //AO Intensity
						}
						AoLayer.radius.overrideState=ActiveRadius.Value; //Activate Radius modification
						if(ActiveRadius.Value)
						{
							AoLayer.radius.value = Radius.Value; //AO Radius
						}
						
						AoLayer.quality.overrideState=ActiveQuality.Value;
						if(ActiveQuality.Value)
						{
							AoLayer.quality.value = AOQuality; // AO Quality choice
						}
						AoLayer.color.overrideState=ActivateColor.Value;
						if(ActivateColor.Value)
						{
							AoLayer.color.value = AOColor.Value; // AO Color
						}
						AoLayer.ambientOnly.overrideState=ActivateAoOnly.Value;
						if(ActivateAoOnly.Value)
						{
							AoLayer.ambientOnly.value = AmbientOnly.Value; // AO Only for forward rendering
						}
						AoLayer.thicknessModifier.overrideState=ActivateThickness.Value;
						if(ActivateThickness.Value)
						{
							AoLayer.thicknessModifier.value = ThicknessModifier.Value; // AO Thikness (MultiScale Mode)
						}

				}
			}	
		}
	}
}

